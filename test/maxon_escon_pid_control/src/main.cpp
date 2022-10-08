/**
 * @file main.cpp
 * @brief
 * @author ZiTe (honmonoh@gmail.com)
 */

// #define INVERSE_DIRECTION

#include "main.h"

static volatile uint32_t systick_delay = 0;

static as5047p_handle_t as5047;
float present_position_deg = 0;
volatile float goal_position_deg = 0;
volatile direction_t encoder_direction = CW;

static float pid_kp = 0.25;
static float pid_ki = 0.008;
static float pid_kd = 0.001;
static float pid_i_term_prev = 0;
static float pid_error_prev = 0;

int main(void)
{
  setup_rcc();
  setup_systick();
  setup_others_gpio();
  // setup_encoder_exti();
  setup_spi();
  setup_usart();
  setup_pwm();
  setup_timer();

  setup_as5047();

  update_present_position();
  goal_position_deg = present_position_deg;

  usart_send_blocking(USART_CONSOLE_INSTANCE, 'O');
  usart_send_blocking(USART_CONSOLE_INSTANCE, 'K');
  usart_send_blocking(USART_CONSOLE_INSTANCE, '\r');
  usart_send_blocking(USART_CONSOLE_INSTANCE, '\n');

  while (1)
  {
    __asm__("nop");
  }

  return 0;
}

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
  systick_set_frequency(1e3, rcc_ahb_frequency); /* Set overflow frequency to 1 kHz. */

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
  /*
   * Setup GPIO.
   */
  gpio_mode_setup(GPIO_SPI_AS5047_MOSI_PIN,
                  GPIO_MODE_AF,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_AS5047_SCK_PIN | GPIO_SPI_AS5047_MISO_PIN | GPIO_SPI_AS5047_MOSI_PIN);

  gpio_set_output_options(GPIO_SPI_AS5047_SCK_MISO_MOSI_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_25MHZ,
                          GPIO_SPI_AS5047_SCK_PIN | GPIO_SPI_AS5047_MOSI_PIN);

  gpio_set_af(GPIO_SPI_AS5047_SCK_MISO_MOSI_PORT,
              GPIO_SPI_AS5047_AF,
              GPIO_SPI_AS5047_SCK_PIN | GPIO_SPI_AS5047_MISO_PIN | GPIO_SPI_AS5047_MOSI_PIN);

  gpio_mode_setup(GPIO_SPI_AS5047_SS_PORT,
                  GPIO_MODE_OUTPUT,
                  GPIO_PUPD_NONE,
                  GPIO_SPI_AS5047_SS_PIN);

  gpio_set_output_options(GPIO_SPI_AS5047_SS_PORT,
                          GPIO_OTYPE_PP,
                          GPIO_OSPEED_25MHZ,
                          GPIO_SPI_AS5047_SS_PIN);

  /* Deselect. */
  gpio_set(GPIO_SPI_AS5047_SS_PORT, GPIO_SPI_AS5047_SS_PIN);

  /*
   * Setup SPI parameters.
   */
  spi_disable(SPI_AS5047_INSTANCE);
  spi_reset(SPI_AS5047_INSTANCE);

  spi_init_master(SPI_AS5047_INSTANCE,
                  SPI_CR1_BAUDRATE_FPCLK_DIV_32,
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
  timer_set_mode(TIMER_ITERATION_INSTANCE,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIMER_ITERATION_INSTANCE);
  timer_continuous_mode(TIMER_ITERATION_INSTANCE);

  auto psc = TIMER_PRESCALER(TIMER_ITERATION_FREQ, TIMER_ITERATION_COUNTER_FREQ);
  auto arr = TIMER_AUTO_RELOAD(TIMER_ITERATION_GOAL_FREQ, TIMER_ITERATION_FREQ, psc);
  timer_set_prescaler(TIMER_ITERATION_INSTANCE, psc);
  timer_set_period(TIMER_ITERATION_INSTANCE, arr);

  timer_enable_irq(TIMER_ITERATION_INSTANCE, TIM_DIER_UIE);
  nvic_enable_irq(TIMER_ITERATION_IRQ);

  // timer_enable_counter(TIMER_ITERATION_INSTANCE);
  timer_disable_counter(TIMER_ITERATION_INSTANCE);
}

static void setup_pwm(void)
{
  gpio_mode_setup(GPIO_PWM_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PWM_PIN);
  gpio_set_output_options(GPIO_PWM_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_25MHZ, GPIO_PWM_PIN);
  gpio_set_af(GPIO_PWM_PORT, GPIO_PWM_AF, GPIO_PWM_PIN);

  timer_set_mode(TIMER_PWM_INSTANCE,
                 TIM_CR1_CKD_CK_INT,
                 TIM_CR1_CMS_EDGE,
                 TIM_CR1_DIR_UP);
  timer_disable_preload(TIMER_PWM_INSTANCE);
  timer_continuous_mode(TIMER_PWM_INSTANCE);
  timer_set_oc_mode(TIMER_PWM_INSTANCE, TIMER_PWM_OC, TIM_OCM_PWM1);

  auto psc = TIMER_PRESCALER(TIMER_PWM_FREQ, TIMER_PWM_COUNTER_FREQ);
  auto arr = TIMER_AUTO_RELOAD(TIMER_PWM_GOAL_FREQ, TIMER_PWM_FREQ, psc);
  auto ccr = TIMER_PWM_CCR(arr, 0.0);
  timer_set_prescaler(TIMER_PWM_INSTANCE, psc);
  timer_set_period(TIMER_PWM_INSTANCE, arr);
  timer_set_oc_value(TIMER_PWM_INSTANCE, TIMER_PWM_OC, ccr);

  timer_enable_oc_output(TIMER_PWM_INSTANCE, TIMER_PWM_OC);
  timer_enable_counter(TIMER_PWM_INSTANCE);
}

static void setup_others_gpio(void)
{
  gpio_mode_setup(GPIO_MOTOR_ENABLE_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_MOTOR_ENABLE_PIN);
  gpio_set_output_options(GPIO_MOTOR_ENABLE_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_MOTOR_ENABLE_PIN);

  gpio_mode_setup(GPIO_MOTOR_DIRECTION_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO_MOTOR_ENABLE_PIN);
  gpio_set_output_options(GPIO_MOTOR_DIRECTION_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_2MHZ, GPIO_MOTOR_ENABLE_PIN);

  set_motor_status(false); /* Disable motor. */
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

static void setup_as5047(void)
{
  as5047p_make_handle(
      &spi_as5047_send,
      &spi_as5047_read,
      &spi_as5047_select,
      &spi_as5047_deselect,
      []()
      { delay_ms(1); },
      &as5047);

  as5047p_config(&as5047, 0b00100101, 0b00000000);
  as5047p_set_zero(&as5047, 0);
}

static void spi_as5047_select(void)
{
  gpio_clear(GPIO_SPI_AS5047_SS_PORT, GPIO_SPI_AS5047_SS_PIN);
}

static void spi_as5047_deselect(void)
{
  /* Wait for SPI transfer complete. */
  while (!(SPI_SR(SPI_AS5047_INSTANCE) & SPI_SR_TXE))
  {
  }
  while ((SPI_SR(SPI_AS5047_INSTANCE) & SPI_SR_BSY))
  {
  }

  gpio_set(GPIO_SPI_AS5047_SS_PORT, GPIO_SPI_AS5047_SS_PIN);
}

static void spi_as5047_send(uint16_t data)
{
  spi_send(SPI_AS5047_INSTANCE, data);
}

static uint16_t spi_as5047_read(void)
{
  spi_send(SPI_AS5047_INSTANCE, 0);                  /* Just for beget clock signal. */
  while ((SPI_SR(SPI_AS5047_INSTANCE) & SPI_SR_BSY)) /* Wait for 'Busy' flag to reset. */
  {
  }

  return spi_read(SPI_AS5047_INSTANCE);
}

static void delay_ns(uint32_t ns)
{
  systick_delay = ns;
  while (systick_delay != 0)
  {
    /* Do nothing and wait. */
  }
}

static void delay_ms(uint32_t ms)
{
  // systick_delay = ms;
  // while (systick_delay > 0)
  // {
  //   __asm__("nop"); /* Do nothing and wait. */
  // }
  uint32_t t = ms * 5000;
  while (t--)
  {
    __asm__("nop"); /* Do nothing and wait. */
  }
}

static void update_present_position(void)
{
  as5047p_get_angle(&as5047, with_daec, &present_position_deg);
}

static void set_pwm_duty_cycle(float duty_cycle)
{
  auto psc = TIMER_PRESCALER(TIMER_PWM_FREQ, TIMER_PWM_COUNTER_FREQ);
  auto arr = TIMER_AUTO_RELOAD(TIMER_PWM_GOAL_FREQ, TIMER_PWM_FREQ, psc);
  auto ccr = TIMER_PWM_CCR(arr, duty_cycle);
  timer_set_oc_value(TIMER_PWM_INSTANCE, TIMER_PWM_OC, ccr);
}

static void set_motor_status(bool enable)
{
  if (enable)
  {
    gpio_set(GPIO_MOTOR_ENABLE_PORT, GPIO_MOTOR_ENABLE_PIN);
  }
  else
  {
    gpio_clear(GPIO_MOTOR_ENABLE_PORT, GPIO_MOTOR_ENABLE_PIN);
  }
}

// static bool get_motor_status(void)
// {
//   auto value = gpio_get(GPIO_MOTOR_ENABLE_PORT, GPIO_MOTOR_ENABLE_PIN);
//   if (value == 1)
//   {
//     return true;
//   }
//   return false;
// }

static void set_motor_direction(direction_t dir)
{
  if (dir == CW)
  {
    gpio_clear(GPIO_MOTOR_DIRECTION_PORT, GPIO_MOTOR_DIRECTION_PIN);
  }
  else if (dir == CCW)
  {
    gpio_set(GPIO_MOTOR_DIRECTION_PORT, GPIO_MOTOR_DIRECTION_PIN);
  }
}

/* -----ISR----- */

void sys_tick_handler(void)
{
  if (systick_delay > 0)
  {
    systick_delay--;
  }
}

void usart2_isr(void)
{
  auto data = usart_recv(USART_CONSOLE_INSTANCE);

  if (data == 0xFF)
  {
    set_motor_status(false);
    timer_disable_counter(TIMER_ITERATION_INSTANCE);
  }
  else if (data == 0xFE)
  {
    set_motor_status(true);
    timer_enable_counter(TIMER_ITERATION_INSTANCE);
  }

  usart_send(USART_CONSOLE_INSTANCE, '.');

  /* Clear 'Read data register not empty' flag. */
  USART_SR(USART_CONSOLE_INSTANCE) &= ~USART_SR_RXNE;
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

void tim1_up_tim10_isr(void)
{
  if (timer_get_flag(TIMER_ITERATION_INSTANCE, TIM_SR_UIF))
  {
    update_present_position();
    auto output = pid_compute(goal_position_deg,
                              present_position_deg,
                              pid_kp,
                              pid_ki,
                              pid_kd,
                              pid_i_term_prev,
                              pid_error_prev,
                              1,
                              100,
                              -100,
                              0,
                              &pid_i_term_prev,
                              &pid_error_prev);
    if (output < 0)
    {
      output *= -1;
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

    set_pwm_duty_cycle((output * 0.8) + 10.0);

    /* Clear 'Update interrupt' flag. */
    timer_clear_flag(TIMER_ITERATION_INSTANCE, TIM_SR_UIF);
  }
}