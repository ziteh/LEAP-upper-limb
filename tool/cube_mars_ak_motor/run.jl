include("akMotor.jl")

position = 0.0
velocity = 0.0
torque = 0.0
kp = 0.0
kd = 0.0
motor = ak10_9_v1_1

function run(pos::Float64, vel::Float64, tor::Float64, kp::Float64, kd::Float64, motor::akMotorParameter)
  return getCanData(pos, vel, tor, kp, kd, motor)
end

function run()
  return getCanData(position, velocity, torque, kp, kd, motor)
end
