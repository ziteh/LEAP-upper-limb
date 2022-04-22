include("pid.jl")

function pid_run(kp, ki, kd, initInput, count, setpointGenerator, systemModel; max=10000, min=-10000)

  time = 1:count
  setPoint = map(setpointGenerator, 1:count)
  input = fill(initInput, count + 2)
  iTerm = fill(0.0, count + 1)
  output = fill(0.0, count)

  for i in time
    (pidOut, lastITerm, lastInput) = pid_compute(
      setPoint[i],
      input[i+1],
      kp, ki, kd,
      iTerm[i],
      input[i],
      max,
      min)

    global iTerm[i+1] = lastITerm
    global output[i] = pidOut

    # PID control is closed-loop control,
    # so the current output(pidOut) will be used as the parameter
    # for the next input(input[i+2]) 
    global input[i+2] = systemModel(lastInput, pidOut)
  end

  return (setPoint, input[3:count+2], output, iTerm[2:count+1])
end