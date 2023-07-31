
// #define SKIP_ALIGNMENT_POSITION

#include <Arduino.h>
#include <SimpleFOC.h>

#define GPIO_DRV8302_INH_A (2)   /* Motor A phase PWM pin. */
#define GPIO_DRV8302_INH_B (3)   /* Motor B phase PWM pin. */
#define GPIO_DRV8302_INH_C (4)   /* Motor C phase PWM pin. */
#define GPIO_DRV8302_EN_GATE (5) /* Motor enable pin. */
#define GPIO_DRV8302_M_PWM (6)   /* PWM mode, when high enables 3PWM mode, else 6PWM mode. */
#define GPIO_DRV8302_M_OC (7)    /* Over-current protection, low to enable. */
#define GPIO_DRV8302_OC_ADJ (8)  /* Over-current limit adjusting analog input. */

/*
 * SPI  SCK: D13 pin.
 * SPI MISO: D12 pin.
 * SPI MOSI: D11 pin.
 */
#define GPIO_AS5047P_SPI_CS (10) /* SPI 'chip select' pin. */

/* T-Motor U8 KV135. */
#define MOTOR_POLE_PAIRS (21)
#define MOTOR_PHASE_RESISTANCE (0.137) /* ohm. */
#define MOTOR_KV (135)                 /* rpm/V. */
#define MOTOR_CURRENT_LIMIT (10)
#define MOTOR_VLOTAGE (24)

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS, MOTOR_PHASE_RESISTANCE);
BLDCDriver3PWM driver = BLDCDriver3PWM(GPIO_DRV8302_INH_A, GPIO_DRV8302_INH_B, GPIO_DRV8302_INH_C, GPIO_DRV8302_EN_GATE);

/* ams AS5047P. */
MagneticSensorSPI positionSensor = MagneticSensorSPI(GPIO_AS5047P_SPI_CS, 14, 0x3FFF);

Commander commander = Commander(Serial);
void onMotor(char *cmd) { commander.motor(&motor, cmd); }

void onReadAngle(char *)
{
  positionSensor.update();
  Serial.println(positionSensor.getAngle(), 3);
}

void drv8302Setup(void);

void setup()
{
  drv8302Setup();

  /* Position sensor setup. */
  positionSensor.spi_mode = SPI_MODE1; /* CPOL=0, CPHA=1. */
  positionSensor.clock_speed = 1e6;    /* 10 MHz max. */
  positionSensor.init();
  motor.linkSensor(&positionSensor);

  /* Driver setup. */
  driver.voltage_power_supply = MOTOR_VLOTAGE;
  driver.init();
  motor.linkDriver(&driver);

  /* Power setup. */
  motor.voltage_limit = MOTOR_VLOTAGE;
  motor.current_limit = MOTOR_CURRENT_LIMIT;

  /* Controller setup. */
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle; /* Close-loop angle/position control. */

  /* Velocity controller setup. */
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  motor.PID_velocity.D = 0.001;
  motor.PID_velocity.output_ramp = 1000; /* Unit in volts/s. */
  motor.LPF_velocity.Tf = 0.01;
  motor.velocity_limit = 20; /* Unit in rad/s. */

  /* Angle/Posision controller setup. */
  motor.P_angle.P = 15;
  motor.P_angle.output_ramp = 5000; /* Acceleration control, unit in rad/s^2. */

  /* Communication setup. */
  Serial.begin(115200);
  motor.useMonitoring(Serial);
  commander.add('M', onMotor, "motor");
  commander.add('S', onReadAngle, "sensor");

#ifdef SKIP_ALIGNMENT_POSITION
  motor.zero_electric_angle = 0;
  motor.sensor_direction = Direction::CCW;
#endif

  motor.init();
  motor.initFOC();

  /* Set target to present position. */
  positionSensor.update();
  motor.target = positionSensor.getAngle();

  _delay(1000);
}

void loop()
{
  motor.loopFOC(); /* Main FOC algorithm. */
  motor.move();    /* Motion control. */

  commander.run();
}

/**
 * @brief DRV8302 specific setup.
 */
void drv8302Setup(void)
{
  pinMode(GPIO_DRV8302_M_PWM, OUTPUT);
  digitalWrite(GPIO_DRV8302_M_PWM, HIGH); /* Enable 3PWM mode. */

  pinMode(GPIO_DRV8302_M_OC, OUTPUT);
  digitalWrite(GPIO_DRV8302_M_OC, LOW); /* Enable over current protection. */

  pinMode(GPIO_DRV8302_OC_ADJ, OUTPUT);
  digitalWrite(GPIO_DRV8302_OC_ADJ, HIGH); /* Set the maximum over current limit possible. */
}
