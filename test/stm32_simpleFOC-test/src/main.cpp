/**
 * @file main.cpp
 * @brief STM32 SimpleFOC basic test.
 */

#include <Arduino.h>
#include <SimpleFOC.h>

// DRV8302 pins connections
// don't forget to connect the common ground pin
#define INH_A (9)
#define INH_B (10)
#define INH_C (11)

#define EN_GATE (8)
#define M_PWM (6)
#define M_OC (5)
#define OC_ADJ (7)

#define VOLTAGE (24)

// motor instance
BLDCMotor motor = BLDCMotor(21, 0.137); // T-Motor U8
// BLDCMotor motor = BLDCMotor(11, 0.216); // Maxon EC90flat
// BLDCMotor motor = BLDCMotor(7, 0.090); // A2212/6T 2200KV

// driver instance
BLDCDriver3PWM driver = BLDCDriver3PWM(INH_A, INH_B, INH_C, EN_GATE);

// encoder instance
// Encoder encoder = Encoder(2, 3, 4000);
// channel A and B callbacks
// void doA() { encoder.handleA(); }
// void doB() { encoder.handleB(); }
// void doIndex() { encoder.handleIndex(); }

// InlineCurrentSensor constructor
//  - shunt_resistor  - shunt resistor value
//  - gain  - current-sense op-amp gain
//  - phA   - A phase adc pin
//  - phB   - B phase adc pin
//  - phC   - C phase adc pin (optional)
// InlineCurrentSense current_sense = InlineCurrentSense(0.005, 12.22, A0, A1, A2);

// Hall sensor instance
// HallSensor(int hallA, int hallB , int cpr, int index)
//  - hallA, hallB, hallC    - HallSensor A, B and C pins
//  - pp                     - pole pairs
// HallSensor sensor = HallSensor(A2, A1, A0, 11);
// Interrupt routine initialization
// channel A and B callbacks
// void doA() { sensor.handleA(); }
// void doB() { sensor.handleB(); }
// void doC() { sensor.handleC(); }

// commander interface
Commander command = Commander(Serial);
void onMotor(char *cmd) { command.motor(&motor, cmd); }

void setup()
{
  // initialize encoder sensor hardware
  // encoder.init();
  // encoder.enableInterrupts(doA, doB, doIndex);
  // encoder.enableInterrupts(doA, doB);

  // check if you need internal pullups
  // Pullup::USE_EXTERN - external pullup added  - default
  // Pullup::USE_INTERN - needs internal arduino pullup
  // encoder.pullup = Pullup::USE_INTERN;

  // link the motor to the sensor
  // motor.linkSensor(&encoder);

  // current sense
  // current_sense.init();
  // motor.linkCurrentSense(&current_sense);
  // sensor.pullup = Pullup::USE_INTERN;
  // sensor.init();
  // hardware interrupt enable
  // sensor.enableInterrupts(doA, doB, doC);
  // motor.linkSensor(&sensor);

  // DRV8302 specific code
  // M_OC  - enable over-current protection
  pinMode(M_OC, OUTPUT);
  digitalWrite(M_OC, LOW);
  // M_PWM  - enable 3pwm mode
  pinMode(M_PWM, OUTPUT);
  digitalWrite(M_PWM, HIGH);
  // OD_ADJ - set the maximum over-current limit possible
  // Better option would be to use voltage divisor to set exact value
  pinMode(OC_ADJ, OUTPUT);
  digitalWrite(OC_ADJ, HIGH);

  // configure driver
  driver.voltage_power_supply = VOLTAGE;
  driver.init();
  motor.linkDriver(&driver);

  // choose FOC modulation
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  // motor.torque_controller = TorqueControlType::voltage;

  // set control loop type to be used
  // motor.controller = MotionControlType::torque;
  motor.controller = MotionControlType::angle_openloop;
  // motor.controller = MotionControlType::velocity_openloop;

  // controller configuration based on the control type
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // motor.PID_velocity.D = 0.005;

  motor.PID_velocity.output_ramp = 50;

  // velocity low pass filtering time constant
  motor.LPF_velocity.Tf = 0.01;

  // angle loop controller
  motor.P_angle.P = 0.1;
  // motor.P_angle.D = 0.02;
  motor.P_angle.output_ramp = 1;

  // angle loop velocity limit
  motor.velocity_limit = 12;
  // default voltage_power_supply
  motor.voltage_limit = VOLTAGE;
  motor.current_limit = 6.5;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialize motor
  motor.init();
  // align encoder and start FOC
  motor.initFOC();

  // set the initial target value
  motor.target = 0;

  // define the motor id
  command.add('M', onMotor, "motor");

  _delay(1000);
}

void loop()
{
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outer loop target
  // velocity, position or voltage
  // if target not set in parameter uses motor.target variable
  motor.move();

  // motor.monitor();

  // user communication
  command.run();

  // sensor.update();
  // Serial.println(sensor.getVelocity());
}