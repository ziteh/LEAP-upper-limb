# julia> include("akMotor.jl")

struct akMotorParameter
  pMax::Float64
  pMin::Float64

  vMax::Float64
  vMin::Float64

  tMax::Float64
  tMin::Float64

  kpMax::Float64
  kpMin::Float64

  kdMax::Float64
  kdMin::Float64
end

ak10_9_v1_1 = akMotorParameter(12.5, -12.5, 50.0, -50.0, 65.0, -65.0, 500.0, 0.0, 5.0, 0.0)

function floatToUint(v::Float64, vMin::Float64, vMax::Float64, bits::Int64)::UInt64
  v = max(min(v, vMax), vMin)

  span = vMax - vMin
  offset = vMin
  fV = ((v - offset) * (2^bits - 1) / span)

  return round(fV, RoundDown)
end

"""
Get the CAN Bus data for given argument.
# Example
```julia-repl
julia> getCanData(6.28, -45.0, 0.0, 2.0, 0.2, ak10_9_v1_1)
```
"""
function getCanData(position::Float64, velocity::Float64, torque::Float64, kp::Float64, kd::Float64, motor::akMotorParameter)
  pos_i = floatToUint(position, motor.pMin, motor.pMax, 16)
  vel_i = floatToUint(velocity, motor.vMin, motor.vMax, 12)
  kp_i = floatToUint(kp, motor.kpMin, motor.kpMax, 12)
  kd_i = floatToUint(kd, motor.kdMin, motor.kdMax, 12)
  tor_i = floatToUint(torque, motor.tMin, motor.tMax, 12)

  canData = Array{UInt8}(undef, 8)
  canData[1] = pos_i >> 8
  canData[2] = pos_i & 0xFF
  canData[3] = vel_i >> 4
  canData[4] = ((vel_i & 0xF) << 4) | (kp_i >> 8)
  canData[5] = kp_i & 0xFF
  canData[6] = kd_i >> 4
  canData[7] = ((kd_i & 0xF) << 4) | (tor_i >> 8)
  canData[8] = tor_i & 0xFF

  return canData
end
