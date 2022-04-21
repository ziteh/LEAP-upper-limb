
function pid_compute(setpoint, input, kp, ki, kd, lsum, lin, max, min)
  error = setpoint - input
  i = ki * error + lsum

  if i > max
    i = max
  elseif i < min
    i = min
  end

  diff = input - lin
  out = kp * error + i - kd * diff

  if out > max
    out = max
  elseif out < min
    out = min
  end

  return (out, i, input)
end