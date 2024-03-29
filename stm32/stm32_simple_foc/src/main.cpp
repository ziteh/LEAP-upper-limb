/**
 * @file main.cpp
 * @brief SIE and SFE joints of LEAP-Up.
 *        STM32 SimpleFOC basic test.
 * @note  https://docs.simplefoc.com/
 */

/*
 * Comment out to use the default UART (i.e. ST-Link UART).
 * For F446 aways comment out.
 * For L432 only comment out when test to ues ST-Link virtual COM port from USB.
 */
// #define SERIAL_UART_INSTANCE 1

#include <Arduino.h>
#include <SimpleFOC.h>

/* Motor selection. */
#define T_MOTOR_U8 /* SIE joint. */
// #define T_MOTOR_U10II   /* SFE joint. */
// #define AK10_9 /* Test. */

#define OPENLOOP /* Comment out to use close-loop control. */
// #define LIMIT_SWITCH /* EXTI. Aways comment out. */

#define BAUDRATE (115200) /* Serial port baudrate. */

/* Motor parameters. */
#if defined(T_MOTOR_U8)
  #define MOTOR_POLE_PAIRS (21)
  #define MOTOR_PHASE_RESISTANCE (0.137) /* Unit in ohm. */
  #define MOTOR_KV (135)                 /* Unit in rpm/V. */
#elif defined(T_MOTOR_U10II)
  #define MOTOR_POLE_PAIRS (21)
  #define MOTOR_PHASE_RESISTANCE (0.101) /* Unit in ohm. */
  #define MOTOR_KV (100)                 /* Unit in rpm/V. */
#elif defined(AK10_9)
  #define MOTOR_POLE_PAIRS (21)
  #define MOTOR_PHASE_RESISTANCE (0.090) /* Unit in ohm. */
  #define MOTOR_KV (100)                 /* Unit in rpm/V. */
#else
  #error No Motor Selected
#endif

/*
 * DRV8302 pins.
 * Select board/MCU from PlatformIO env (platformio.ini).
 * The pins named without "P" prefix are Arduino pin number，otherwise are STM32 pin name.
 */
#if defined(NUCLEO_L432KC)
  #define INH_A (9)  /* PA8. */
  #define INH_B (A2) /* PA3. */
  #define INH_C (A1) /* PA1. */

  #define EN_GATE (2) /* PA12. */
  #define OC_ADJ (3)  /* PB0. */
  #define M_OC (4)    /* PB7. */
  #define M_PWM (5)   /* PB6. */

  #define LIMIT_SWITCH_PIN (A0) /* PA0. */

  /*
   * SPI SCLK: D13 pin (PB3).
   * SPI MISO: D12 pin (PB4).
   * SPI MOSI: D11 pin (PB5).
   */
  #define AS5047P_SPI_CS (10) /* PA11. */
#elif defined(NUCLEO_F446RE) || defined(NUCLEO_F401RE)
  #define INH_A (6)
  #define INH_B (5)
  #define INH_C (3)

  #define EN_GATE (8)
  #define OC_ADJ (7)
  #define M_OC (PB13)
  #define M_PWM (PB14)

  #define LIMIT_SWITCH_PIN (A0)

  /*
   * SPI SCLK: D13 pin.
   * SPI MISO: D12 pin.
   * SPI MOSI: D11 pin.
   */
  #define AS5047P_SPI_CS (10)
#else
  #error No Board Selected
#endif

/* Power. */
#define VOLTAGE_SUPPLY (22.2) /* Unit in V. */
#define CURRENT_LIMIT (10)    /* Unit in A. */

/* AS5047P encoder. */
#define AS5047P_REG_ANGLECOM (0x3FFF) /* Measured angle with dynamic angle error compensation(DAEC). */
#define AS5047P_REG_ANGLEUNC (0x3FFE) /* Measured angle without DAEC. */

#define LIMIT_T0_ZERO_DIFF (9) /* The diff between limit switch triggered and zero position in Rad. */

HardwareSerial Serial1(USART1);

bool limited = false;
bool homing = false;

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);
MagneticSensorSPI angleSensor = MagneticSensorSPI(AS5047P_SPI_CS, 14, AS5047P_REG_ANGLECOM);

Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void onReadAngle(char *)
{
#ifndef OPENLOOP
  angleSensor.update();
  float angle = angleSensor.getAngle();

  angle -= motor.sensor_offset;
  // angle *= -1;

  char sign;
  if (angle >= 0)
  {
    sign = '+';
  }
  else
  {
    sign = '-';
  }

  angle = fabs(angle); /* Absolute value. */

  /* Send. */
  Serial.printf("%c%d%d%d.%d%d%d\r\n",
                sign,
                (int)((int)(angle) / 100 % 10),
                (int)((int)(angle) / 10 % 10),
                (int)((int)(angle) / 1 % 10),
                (int)((int)(angle * 10) / 1 % 10),
                (int)((int)(angle * 100) / 1 % 10),
                (int)((int)(angle * 1000) / 1 % 10));

#endif
}

void onLimitSwitchTriggered(void);
void drv8302Setup(void);

void setup()
{
  /*
   * Communication setup.
   * ME0: Disable, ME1: Enable, ME: Check enable state
   * E3.14: move to 3.14 rad, E-2: move to -2 rad
   * A: Read present position
   * newline "\n (LF)" at the end is required.
   * https://docs.simplefoc.com/commander_interface
   */
  Serial.begin(BAUDRATE);
  motor.useMonitoring(Serial);
  command.add('M', onMotor, "motor");
  command.add('A', onReadAngle, "angle");

#ifndef OPENLOOP
  /* Configure angle/Position sensor. https://docs.simplefoc.com/magnetic_sensor_spi */
  angleSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  angleSensor.clock_speed = 1e6;    /* 10 MHz max. */
  angleSensor.init();
  motor.linkSensor(&angleSensor);
#endif

  /* Configure driver. https://docs.simplefoc.com/bldcdriver3pwm */
  drv8302Setup();
  driver.pwm_frequency = 20000;
  driver.voltage_power_supply = VOLTAGE_SUPPLY;
  driver.init();
  motor.linkDriver(&driver);

  /* Motor code: https://docs.simplefoc.com/bldcmotor. */

  /* Configure motor parameters. */
  // motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  // motor.KV_rating = MOTOR_KV * 1.5f; /* SimpleFOC suggest to set the KV value provided to the library to 50-70% higher than the one given in the datasheet.. */
  motor.voltage_limit = VOLTAGE_SUPPLY;
  motor.current_limit = CURRENT_LIMIT;
  // motor.motion_downsample = 5;

  /* Algorithms and controllers setup. */
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM; /* SVPWM. */
  motor.torque_controller = TorqueControlType::voltage;     /* No current sensing so only voltage controller. */
#ifdef OPENLOOP
  motor.controller = MotionControlType::angle_openloop;
#else
  motor.controller = MotionControlType::angle;
#endif

  /* Velocity control loop setup. */
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // motor.PID_velocity.D = 0.001;
  // motor.PID_velocity.output_ramp = 500; /* Unit in volts/s. */
  motor.LPF_velocity.Tf = 0.01;
  // motor.velocity_limit = 15; /* Unit in rad/s. */

  /* Angle/Position control loop setup. */
  motor.P_angle.P = 5;
  // motor.P_angle.I = 0.5;
  // motor.P_angle.D = 0.05;
  // motor.P_angle.output_ramp = 500; /* Acceleration limit(?), unit in rad/s^2. */

  motor.init();    /* Initialize motor. */
  motor.initFOC(); /* Start FOC and aligh encoder. */

#ifndef OPENLOOP
  angleSensor.update();
  motor.target = angleSensor.getAngle(); /* Set the initial target value. */
#else
  motor.target = 0;
#endif

  Serial.println(motor.target, 3);

#ifdef LIMIT_SWITCH
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLDOWN);
  _delay(10);

  /* Find limit switch triggered position. */
  homing = true;
  Serial.printf("Find limit switch... ");
  motor.controller = MotionControlType::velocity;
  // motor.velocity_limit = 20; /* Unit in rad/s. */
  motor.enable();
  while (digitalRead(LIMIT_SWITCH_PIN) != LOW)
  {
    motor.move(12.5);
    motor.loopFOC(); /* Main FOC algorithm. */
  }

  /* Limit switch triggered posision finded. */
  motor.disable();
  _delay(10);

  /* Update zero position offset. */
  angleSensor.update();
  motor.sensor_offset = +angleSensor.getAngle() - LIMIT_T0_ZERO_DIFF;

  /* Move to zero (zero position != limit switch triggered position). */
  Serial.printf("Go to zero... ");
  motor.controller = MotionControlType::angle;
  motor.target = 0;
  motor.enable();
  while (1)
  {
    angleSensor.update();
    if (angleSensor.getAngle() - motor.sensor_offset < 0.25) /* 0.1 allowable error. */
    {
      break; /* Move done. */
    }

    motor.move();
    motor.loopFOC(); /* Main FOC algorithm. */
  }

  homing = false;  /* Reset flag. */
  limited = false; /* Reset flag. */
  Serial.println("Done.");

  /* Configure limit switch interrupt. */
  // attachInterrupt(LIMIT_SWITCH_PIN, onLimitSwitchTriggered, FALLING);
  // attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), onLimitSwitchTriggered, FALLING);
#endif
  // motor.velocity_limit = 15; /* Unit in rad/s. */

  Serial.println("All Ready!");
  _delay(1000);
}

void loop()
{
  motor.loopFOC(); /* Main FOC algorithm. */
  motor.move();    /* Motion control. */

  command.run();

#ifdef LIMIT_SWITCH
  if (digitalRead(LIMIT_SWITCH_PIN) == LOW)
  {
    motor.disable();
    limited = true;
    Serial.println("LST"); /* Limit switch triggered. */
  }
#endif
}

/*
 * DRV8302 specific setup.
 * https://docs.simplefoc.com/drv8302_example
 */
void drv8302Setup(void)
{
  /*
   * M_PWM: Mode selection pin for PWM input configuration.
   * - LOW: 6 PWM mode.
   * - HIGH: 3 PWM mode, only INH_x pins required.
   */
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);

  /*
   * M_OC: Mode selection pin for over-current protection options.
   * - LOW: the gate driver will operate in a cycle-by-cycle current limiting mode.
   * - HIGH: the gate driver will shutdown the channel which detected an over-current event.
   */
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);

  /*
   * OD_ADJ: Overcurrent trip set pin.
   * Set HIGH for the maximum over-current limit possible,
   * better option would be to use voltage divisor to set exact value.
   */
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);
}

void onLimitSwitchTriggered(void)
{
  /* Debounce. */
  // for (int i = 0; i < 3; i++)
  // {
  //   /* Delay. */
  //   for (unsigned int j = 0; j < 5e4; j++)
  //   {
  //     __ASM("nop");
  //   }

  //   if (digitalRead(LIMIT_SWITCH_PIN) == HIGH)
  //   {
  //     return;
  //   }
  // }

  if (homing)
  {
    return;
  }

  /* Delay. */
  for (unsigned int i = 0; i < 5e4; i++)
  {
    __ASM("nop");
  }

  if (digitalRead(LIMIT_SWITCH_PIN) == LOW)
  {
    limited = true;

    motor.disable();
    // limited = true;
    Serial.println("LST"); /* Limit switch triggered. */
  }
}
