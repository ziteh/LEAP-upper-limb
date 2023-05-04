/**
 * @file main.cpp
 * @brief maxon ESCON motor controller PID angle/position control program.
 */

#include "main.h"

/* Timer macros. */
#define TIMER_PRESCALER(f_tim, f_cnt) (f_tim / f_cnt - 1)                          /* PSC. */
#define TIMER_AUTO_RELOAD(f_goal, f_tim, psc) ((f_tim / ((psc + 1) * f_goal)) - 1) /* ARR. */
#define TIMER_PWM_CCR(arr, dc_goal) ((arr + 1) * dc_goal / 100.0)                  /* CCR. */

as5047p_handle_t as5047;

static volatile uint32_t systick_delay = 0;
float present_position_deg = 0;
volatile float goal_position_deg = 0;
volatile direction_t encoder_direction = CW;

/* PID controller paremeters. */
static float pid_kp = 1;
static float pid_ki = 0.002;
static float pid_kd = 0.001;
static float pid_i_term_prev = 0;
static float pid_error_prev = 0;

int main(void)
{
  /* Init. */
  setup_rcc();
  setup_systick();
  setup_others_gpio();
  // setup_encoder_exti();
  setup_spi();
  setup_usart();
  setup_pwm();
  setup_timer();
  setup_as5047();
  setup_limit_switch_exti();
  /* The first reading AS5047P value may be 0. */
  for (int i = 0; i < 3; i++)
  {
    update_present_position();
    delay_ms(200);
  }
  goal_position_deg = present_position_deg;

  usart_send_blocking(USART_CONSOLE_INSTANCE, 'O');
  usart_send_blocking(USART_CONSOLE_INSTANCE, 'K');
  usart_send_blocking(USART_CONSOLE_INSTANCE, '\r');
  usart_send_blocking(USART_CONSOLE_INSTANCE, '\n');

  // printf("OK\r\n");

  while (1)
  {
    __asm__("nop"); /* Do nothing, wait for ISRs. */
  }

  return 0;
}

/**
 * @brief PID controller algorithm.
 */
static float pid_compute(float set_value,
                         float actual_value,
                         float kp,
                         float ki,
                         float kd,
                         float i_term_prev,
                         float error_prev,
                         float iteration_time,
                         float max,
                         float min,
                         float bias,
                         float *i_term_out,
                         float *error_out)
{
  auto error = set_value - actual_value;

  auto p_term = kp * error;
  auto i_term = ki * (i_term_prev + error * iteration_time);
  auto d_term = kd * (error - error_prev) / iteration_time;

  /* I-term limit. */
  if (i_term > max)
  {
    i_term = max;
  }
  if (i_term < min)
  {
    i_term = min;
  }

  auto output = p_term + i_term + d_term + bias;

  /* Final output limit. */
  if (output > max)
  {
    output = max;
  }
  if (output < min)
  {
    output = min;
  }

  /* Output. */
  i_term_out = &i_term;
  error_out = &error;
  return output;
}

static void setup_rcc(void)
{
  rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

  rcc_periph_clock_enable(RCC_GPIOA);
  rcc_periph_clock_enable(RCC_GPIOB);
  rcc_periph_clock_enable(RCC_SYSCFG); /* For EXTI. */
  rcc_periph_clock_enable(RCC_USART2);
  rcc_periph_clock_enable(RCC_SPI1);
  rcc_periph_clock_enable(RCC_TIM2);
  rcc_periph_clock_enable(RCC_TIM10);

  rcc_periph_reset_pulse(RST_TIM2);
  rcc_periph_reset_pulse(RST_TIM10);
}

static void setup_systick(void)
{
  systick_set_frequency(1e6, rcc_ahb_frequency); /* Set overflow frequency to 1 MHz. */

  systick_interrupt_enable();
  systick_counter_enable();
}

static void setup_usart(void)
{
  /* Set Tx and Rx pin to push-pull alternate function. */
  gpio_mode_setup(GPIO_USART_CONSOLE_TXRX_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_USART_CONSOLE_TX_PIN | GPIO_USART_CONSOLE_RX_PIN);

  gpio_set_output_options(GPIO_USART_CONSOLE_TXRX_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_50MHZ,
                          GPIO_USART_CONSOLE_TX_PIN | GPIO_USART_CONSOLE_RX_PIN);

  gpio_set_af(GPIO_USART_CONSOLE_TXRX_PORT,
              GPIO_USART_CONSOLE_AF,
              GPIO_USART_CONSOLE_TX_PIN | GPIO_USART_CONSOLE_RX_PIN);

  /* Config USART params. */
  usart_set_baudrate(USART_CONSOLE_INSTANCE, USART_CONSOLE_BAUDRATE);
  usart_set_databits(USART_CONSOLE_INSTANCE, 8);
  usart_set_stopbits(USART_CONSOLE_INSTANCE, USART_STOPBITS_1);
  usart_set_parity(USART_CONSOLE_INSTANCE, USART_PARITY_NONE);
  usart_set_flow_control(USART_CONSOLE_INSTANCE, USART_FLOWCONTROL_NONE);
  usart_set_mode(USART_CONSOLE_INSTANCE, USART_MODE_TX_RX);

  /* Setup interrupt. */
  usart_enable_rx_interrupt(USART_CONSOLE_INSTANCE); /* Enable receive interrupt. */
  nvic_enable_irq(USART_CONSOLE_IRQ);

  usart_enable(USART_CONSOLE_INSTANCE);
}

static void setup_spi(void)
{
  /* Setup GPIO. */
  gpio_mode_setup(GPIO_SPI_AS5047_SCK_MISO_MOSI_PORT,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_AS5047_SCK_PIN | GPIO_SPI_AS5047_MISO_PIN | GPIO_SPI_AS5047_MOSI_PIN);

  gpio_set_output_options(GPIO_SPI_AS5047_SCK_MISO_MOSI_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_50MHZ,
                          GPIO_SPI_AS5047_SCK_PIN | GPIO_SPI_AS5047_MOSI_PIN);

  gpio_set_af(GPIO_SPI_AS5047_SCK_MISO_MOSI_PORT,
              GPIO_SPI_AS5047_AF,
              GPIO_SPI_AS5047_SCK_PIN | GPIO_SPI_AS5047_MISO_PIN | GPIO_SPI_AS5047_MOSI_PIN);

  /* In master mode, control SS pin by user instead of AF. */
  gpio_mode_setup(GPIO_SPI_AS5047_SS_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_AS5047_SS_PIN);

  gpio_set_output_options(GPIO_SPI_AS5047_SS_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_25MHZ,
                          GPIO_SPI_AS5047_SS_PIN);

  /* Deselect. */
  spi_as5047_deselect();

  /* Setup SPI parameters. */
  spi_disable(SPI_AS5047_INSTANCE);
  spi_reset(SPI_AS5047_INSTANCE);

  spi_init_master(SPI_AS5047_INSTANCE,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_64,
                  SPI_CR1_CPOL_CLK_TO_0_WHEN_IDLE, /* CPOL=0. */
                  SPI_CR1_CPHA_CLK_TRANSITION_2,   /* CPHA=1. */
                  SPI_CR1_DFF_16BIT,
                  SPI_CR1_MSBFIRST);
  spi_set_full_duplex_mode(SPI_AS5047_INSTANCE);

  /* For master device, set SSM=1 and SSI=1 to prevent any MODF error. */
  spi_enable_software_slave_management(SPI_AS5047_INSTANCE); /* Set SSM to 1. */
  spi_set_nss_high(SPI_AS5047_INSTANCE);                     /* Set SSI to 1. */

  spi_enable(SPI_AS5047_INSTANCE);
}

static void setup_timer(void)
{
  /* Configure timer mode. */
  timer_set_mode(TIMER_ITERATION_INSTANCE,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIMER_ITERATION_INSTANCE);
  timer_continuous_mode(TIMER_ITERATION_INSTANCE);

  /* Setup frequency by PSC and ARR registers. */
  auto psc = TIMER_PRESCALER(TIMER_ITERATION_FREQ, TIMER_ITERATION_COUNTER_FREQ);
  auto arr = TIMER_AUTO_RELOAD(TIMER_ITERATION_GOAL_FREQ, TIMER_ITERATION_FREQ, psc);
  timer_set_prescaler(TIMER_ITERATION_INSTANCE, psc);
  timer_set_period(TIMER_ITERATION_INSTANCE, arr);

  /* Enable interrupt. */
  timer_enable_irq(TIMER_ITERATION_INSTANCE, TIM_DIER_UIE); /* Select 'Update interrupt'. */
  nvic_enable_irq(TIMER_ITERATION_IRQ);

  /* Init disable. */
  timer_disable_counter(TIMER_ITERATION_INSTANCE);
}

static void setup_pwm(void)
{
  /* Setup GPIO. */
  gpio_mode_setup(GPIO_PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PWM_PIN);
  gpio_set_output_options(GPIO_PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PWM_PIN);
  gpio_set_af(GPIO_PWM_PORT, GPIO_PWM_AF, GPIO_PWM_PIN);

  /* Configure timer mode. */
  timer_set_mode(TIMER_PWM_INSTANCE,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIMER_PWM_INSTANCE);
  timer_continuous_mode(TIMER_PWM_INSTANCE);
  timer_set_oc_mode(TIMER_PWM_INSTANCE, TIMER_PWM_OC, TIM_OCM_PWM1);

  /* Setup frequency and duty cycle by PSC, ARR and CCR registers. */
  auto psc = TIMER_PRESCALER(TIMER_PWM_FREQ, TIMER_PWM_COUNTER_FREQ);
  auto arr = TIMER_AUTO_RELOAD(TIMER_PWM_GOAL_FREQ, TIMER_PWM_FREQ, psc);
  auto ccr = TIMER_PWM_CCR(arr, 0.0); /* Init set duty cycle to 0%. */
  timer_set_prescaler(TIMER_PWM_INSTANCE, psc);
  timer_set_period(TIMER_PWM_INSTANCE, arr);
  timer_set_oc_value(TIMER_PWM_INSTANCE, TIMER_PWM_OC, ccr);

  /* Enable. */
  timer_enable_oc_output(TIMER_PWM_INSTANCE, TIMER_PWM_OC);
  timer_enable_counter(TIMER_PWM_INSTANCE);
}

static void setup_others_gpio(void)
{
  /* Motor enable pin. */
  gpio_mode_setup(GPIO_MOTOR_ENABLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_MOTOR_ENABLE_PIN);
  gpio_set_output_options(GPIO_MOTOR_ENABLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_MOTOR_ENABLE_PIN);
  set_motor_enable(false); /* Init disable motor. */

  /* Motor direction pin. */
  gpio_mode_setup(GPIO_MOTOR_DIRECTION_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_MOTOR_DIRECTION_PIN);
  gpio_set_output_options(GPIO_MOTOR_DIRECTION_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_MOTOR_DIRECTION_PIN);
}

static void setup_encoder_exti(void)
{
  gpio_mode_setup(GPIO_ENCODER_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_PULLUP,
                  GPIO_ENCODER_A_PIN | GPIO_ENCODER_B_PIN | GPIO_ENCODER_I_PIN);

  exti_select_source(EXTI_ENCODER_A, GPIO_ENCODER_PORT);
  exti_select_source(EXTI_ENCODER_B, GPIO_ENCODER_PORT);
  exti_select_source(EXTI_ENCODER_I, GPIO_ENCODER_PORT);

  exti_set_trigger(EXTI_ENCODER_A, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI_ENCODER_B, EXTI_TRIGGER_RISING);
  exti_set_trigger(EXTI_ENCODER_I, EXTI_TRIGGER_RISING);

  exti_enable_request(EXTI_ENCODER_A);
  exti_enable_request(EXTI_ENCODER_B);
  exti_enable_request(EXTI_ENCODER_I);

  nvic_enable_irq(ENCODER_A_IRQ);
  nvic_enable_irq(ENCODER_B_IRQ);
  nvic_enable_irq(ENCODER_I_IRQ);
}

static void setup_limit_switch_exti(void)
{
  gpio_mode_setup(GPIO_LIMIT_SWITCH_PORT,
                  GPIO_MODE_INPUT,
                  GPIO_PUPD_PULLDOWN,
                  GPIO_LIMIT_SWITCH_PIN);

  exti_select_source(EXTI_LIMIT_SWITCH, GPIO_LIMIT_SWITCH_PORT);
  exti_set_trigger(EXTI_LIMIT_SWITCH, EXTI_TRIGGER_FALLING);

  exti_enable_request(EXTI_LIMIT_SWITCH);
  nvic_enable_irq(LIMIT_SWITCH_IRQ);
}

/**
 * @brief Set the up as5047 sensor.
 * @note Ref: https://github.com/ziteh/as5047p-driver
 */
static void setup_as5047(void)
{
  as5047p_make_handle(&spi_as5047_send,
                      &spi_as5047_read,
                      &spi_as5047_select,
                      &spi_as5047_deselect,
                      &spi_as5047_delay,
                      &as5047);

  as5047p_config(&as5047, 0b00100101, 0b00000000);
  as5047p_set_zero(&as5047, AS5047_ZERO_POSITION);
}

/**
 * @brief Callback function for SPI select.
 */
void spi_as5047_select(void)
{
  /* Low to select. */
  gpio_clear(GPIO_SPI_AS5047_SS_PORT, GPIO_SPI_AS5047_SS_PIN);
}

/**
 * @brief Callback function for SPI deselect.
 */
void spi_as5047_deselect(void)
{
  /* High to deselect. */
  gpio_set(GPIO_SPI_AS5047_SS_PORT, GPIO_SPI_AS5047_SS_PIN);
}

/**
 * @brief Callback function for SPI send data.
 */
void spi_as5047_send(uint16_t data)
{
  spi_send(SPI_AS5047_INSTANCE, data);

  /*
   * Wait for SPI transmit complete.
   * Ref: https://controllerstech.com/spi-using-registers-in-stm32/.
   */
  while (!(SPI_SR(SPI_AS5047_INSTANCE) & SPI_SR_TXE)) /* Wait for 'Transmit buffer empty' flag to set. */
  {
  }
  while ((SPI_SR(SPI_AS5047_INSTANCE) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }
}

/**
 * @brief Callback function for SPI read data.
 */
uint16_t spi_as5047_read(void)
{
  // uint16_t data = spi_xfer(SPI_AS5047_INSTANCE, 0);
  spi_send(SPI_AS5047_INSTANCE, 0); /* Just for beget clock signal. */
  uint16_t data = spi_read(SPI_AS5047_INSTANCE);

  while ((SPI_SR(SPI_AS5047_INSTANCE) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }

  return data;
}

/**
 * @brief For 't_CSn': High time of CSn between two transmissions, >350 ns.
 */
void spi_as5047_delay(void)
{
  delay_ms(1);
}

/* BUG: SysTick System Timer Does Not Generate Interrupts (https://developer.arm.com/documentation/ka002893/latest). */
static void delay_ns(uint32_t ns)
{
  systick_delay = ns;
  while (systick_delay != 0)
  {
    __asm__("nop"); /* Do nothing and wait. */
  }
}

static void delay_ms(uint32_t ms)
{
  /* BUG. */
  // for (uint16_t i = 0; i < 1000; i++)
  // {
  //   delay_ns(ms);
  // }

  for (uint32_t i = ms * 5000; i > 0; i--)
  {
    __asm__("nop"); /* Do nothing and wait. */
  }
}

static void update_present_position(void)
{
  int8_t error = as5047p_get_angle(&as5047, without_daec, &present_position_deg);
  if (error != 0)
  {
    usart_send_blocking(USART_CONSOLE_INSTANCE, 'E');
  }
}

static void set_pwm_duty_cycle(float duty_cycle)
{
  auto psc = TIMER_PRESCALER(TIMER_PWM_FREQ, TIMER_PWM_COUNTER_FREQ);
  auto arr = TIMER_AUTO_RELOAD(TIMER_PWM_GOAL_FREQ, TIMER_PWM_FREQ, psc);
  auto ccr = TIMER_PWM_CCR(arr, duty_cycle);
  timer_set_oc_value(TIMER_PWM_INSTANCE, TIMER_PWM_OC, ccr);
}

static void set_motor_enable(bool enable)
{
  if (enable)
  {
    gpio_set(GPIO_MOTOR_ENABLE_PORT, GPIO_MOTOR_ENABLE_PIN); /* High to enable. */
  }
  else
  {
    gpio_clear(GPIO_MOTOR_ENABLE_PORT, GPIO_MOTOR_ENABLE_PIN); /* Low to disable. */
  }
}

static void set_motor_direction(direction_t dir)
{
  if (dir == CW)
  {
    gpio_clear(GPIO_MOTOR_DIRECTION_PORT, GPIO_MOTOR_DIRECTION_PIN); /* Low for CW. */
  }
  else if (dir == CCW)
  {
    gpio_set(GPIO_MOTOR_DIRECTION_PORT, GPIO_MOTOR_DIRECTION_PIN); /* High for CCW. */
  }
  /* Else do nothing. */
}

/* BUG. */
/**
 * @brief For 'printf()'.
 */
// int _write(int file, char *ptr, int len)
// {
//   int i;

//   if (file == 1)
//   {
//     for (i = 0; i < len; i++)
//     {
//       usart_send_blocking(USART_CONSOLE_INSTANCE, ptr[i]);
//     }
//     return i;
//   }

//   errno = EIO;
//   return -1;
// }

/* -----ISRs----- */

/**
 * @brief Main communication procedures. USART2 ISR.
 */
void usart2_isr(void)
{
  uint8_t data = usart_recv(USART_CONSOLE_INSTANCE);

  if (data == 0xFF) /* Disable motor. */
  {
    set_motor_enable(false);
    timer_disable_counter(TIMER_ITERATION_INSTANCE);
  }
  else if (data == 0xFE) /* Enable motor. */
  {
    timer_enable_counter(TIMER_ITERATION_INSTANCE);
    delay_ms(250);
    set_motor_enable(true);
  }
  else if (data == 0xFA) /* Read raw position. */
  {
    uint16_t position;
    int8_t error = as5047p_get_position(&as5047, without_daec, &position);

    if (error != 0)
    {
      /* Error occurred. */
      usart_send_blocking(USART_CONSOLE_INSTANCE, 0xFE);
      usart_send_blocking(USART_CONSOLE_INSTANCE, 0xFE);
    }
    else
    {
      usart_send_blocking(USART_CONSOLE_INSTANCE, (position >> 8) & 0x00FF); /* High 8 bits. */
      usart_send_blocking(USART_CONSOLE_INSTANCE, position & 0x00FF);        /* Low 8 bits. */
    }
  }
  else /* Control. */
  {
    if (data > 0 && data < 70)
    {
      goal_position_deg = (float)data;
    }
  }

  usart_send_blocking(USART_CONSOLE_INSTANCE, '\n');

  /* Clear 'Read data register not empty' flag. */
  USART_SR(USART_CONSOLE_INSTANCE) &= ~USART_SR_RXNE;
}

/**
 * @brief Main loop, running PID algorithm and control motor. Timer10 ISR.
 */
void tim1_up_tim10_isr(void)
{
  if (timer_get_flag(TIMER_ITERATION_INSTANCE, TIM_SR_UIF)) /* Check 'Update interrupt' flag. */
  {
    /* Running PID algo. */
    update_present_position();
    auto output = pid_compute(goal_position_deg,
                              present_position_deg,
                              pid_kp,
                              pid_ki,
                              pid_kd,
                              pid_i_term_prev,
                              pid_error_prev,
                              (1.0 / TIMER_ITERATION_GOAL_FREQ),
                              PWM_PID_DC_MAX,
                              PWM_PID_DC_MIN,
                              PWM_PID_BIAS,
                              &pid_i_term_prev,
                              &pid_error_prev);

    /* Direction output. */
    if (output < 0)
    {
      output *= -1; /* Absolute value. */

#ifdef INVERSE_DIRECTION
      set_motor_direction(CW);
#else
      set_motor_direction(CCW);
#endif
    }
    else
    {
#ifdef INVERSE_DIRECTION
      set_motor_direction(CCW);
#else
      set_motor_direction(CW);
#endif
    }

    /* Update PWM duty cycle. */
    set_pwm_duty_cycle(output + PWM_DC_OFFSET);

    /* Clear 'Update interrupt' flag. */
    timer_clear_flag(TIMER_ITERATION_INSTANCE, TIM_SR_UIF);
  }
}

/**
 * @brief SysTick ISR.
 */
void sys_tick_handler(void)
{
  if (systick_delay != 0)
  {
    systick_delay--;
  }
}

/* Encoder A. */
void exti4_isr(void)
{
  uint16_t b = gpio_get(GPIO_ENCODER_PORT, GPIO_ENCODER_B_PIN);
  if (b == 0)
  {
    encoder_direction = CW;
    present_position_deg++;
  }
  else
  {
    encoder_direction = CCW;
    present_position_deg--;
  }
  exti_reset_request(EXTI_ENCODER_A);
}

/* Encoder B. */
void exti9_5_isr(void)
{
  exti_reset_request(EXTI_ENCODER_B);
}

/* Encoder I. */
void exti3_isr(void)
{
  exti_reset_request(EXTI_ENCODER_I);
}

/* Limit switch. */
void exti0_isr(void)
{
  /* Clear flag. */
  exti_reset_request(EXTI_LIMIT_SWITCH);

  /* Debounce. */
  for (int i = 0; i < 3; i++)
  {
    delay_ms(10);
    if (gpio_get(GPIO_LIMIT_SWITCH_PORT, GPIO_LIMIT_SWITCH_PIN) != 0)
    {
      return;
    }
  }
  
  set_motor_enable(false);
  set_pwm_duty_cycle(0);

  /* Limit switch triggered. */
  usart_send_blocking(USART_CONSOLE_INSTANCE, 'L');
  usart_send_blocking(USART_CONSOLE_INSTANCE, 'S');
  usart_send_blocking(USART_CONSOLE_INSTANCE, 'T');
  usart_send_blocking(USART_CONSOLE_INSTANCE, '\r');
  usart_send_blocking(USART_CONSOLE_INSTANCE, '\n');
}
