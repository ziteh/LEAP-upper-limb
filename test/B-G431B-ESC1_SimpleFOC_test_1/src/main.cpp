/**
 * @file main.cpp
 * @brief ST B-G431B-ESC1 board SimpleFOC test.
 * @note Ref: https://community.simplefoc.com/t/b-g431b-esc1-beginner-guide-i2c-guide/515
 */

#include <Arduino.h>
#include <SimpleFOC.h>

#define BAUDRATE (115200)

#define MOTOR_POLE_PAIRS (21)
#define MOTOR_PHASE_RESISTANCE (0.137) // Unit in ohm.
#define MOTOR_KV (135)
#define MOTOR_CURRENT_LIMIT (5) // Unit in A.
#define MOTOR_VOLTAGE (22.2)    // Unit in V.

BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH,
                                       A_PHASE_UL,
                                       A_PHASE_VH,
                                       A_PHASE_VL,
                                       A_PHASE_WH,
                                       A_PHASE_WL);

Commander commander = Commander(Serial2);
void onMotor(char *cmd) { commander.motor(&motor, cmd); }

void setup()
{
  // Driver setup.
  driver.voltage_power_supply = MOTOR_VOLTAGE;
  // driver.pwm_frequency = 35000;
  // driver.dead_zone = 0.01; // x100%
  driver.init();
  motor.linkDriver(&driver);

  // Motor parameters setup.
  motor.phase_resistance = MOTOR_PHASE_RESISTANCE;
  // motor.KV_rating = MOTOR_KV * 1.5; /* SimpleFOC suggest to set the KV value provided to the library to 50-70% higher than the one given in the datasheet. */
  motor.voltage_limit = MOTOR_VOLTAGE;
  motor.current_limit = MOTOR_CURRENT_LIMIT;

  // Algorithms and controllers setup.
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  motor.torque_controller = TorqueControlType::voltage;
  motor.controller = MotionControlType::angle_openloop;
  // motor.controller = MotionControlType::velocity_openloop;

  // Velocity control loop setup.
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  // motor.PID_velocity.D = 0.001;
  // motor.PID_velocity.output_ramp = 1000; // Unit in volts/s.
  motor.LPF_velocity.Tf = 0.01;
  motor.velocity_limit = 50; // Unit in rad/s.

  // Angle/Position control loop setup.
  motor.P_angle.P = 5;
  motor.P_angle.I = 1;
  motor.P_angle.output_ramp = 500; // Acceleration limit, unit in rad/s^2.

  // Communication setup.
  Serial2.begin(BAUDRATE);
  motor.useMonitoring(Serial2);
  commander.add('M', onMotor, "motor");

  // Init.
  // motor.target = 0; // Set the initial target.
  motor.init();
  motor.initFOC();

  // Serial2.println("Ready\n");
  delay(1000);
}

void loop()
{
  motor.loopFOC(); // Main FOC algorithm.
  motor.move();    // Motion control.

  commander.run();
}
