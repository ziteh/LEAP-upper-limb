include("pid_runner.jl")

using Plots
using Printf

# Generate setpoint data
function defaultSetPointGenerator(t)
  if t > 200
    return 50
  elseif t > 100
    return -5
  elseif t > 50
    return 100
  else
    return 25
  end
end

# Describe how the system responds to the PID output. 
defaultSystemModel(lastInput, feedback) = lastInput + feedback * 0.3 + 1

function pidPlot(kp, ki, kd, max=1000.0, min=-1000.0; count=300, initInput=25.0, setPointGenerator=defaultSetPointGenerator, systemModel=defaultSystemModel)
  (setPoint, input, output, iTerm) = pid_run(kp, ki, kd, initInput, count, setPointGenerator, systemModel, max=max, min=min)

  # Print info
  println("--- PID Plot ---")
  @printf("kp: %f, ki: %f, Kd: %f, max: %f, min: %f", kp, ki, kd, max, min)
  println("")

  # Plot out
  t = 1:count
  plotMain = plot(t, input, label="Input")
  plot!(t, setPoint, label="Setpoint", linestyle=:dash)
  plotSub = plot(t, output, label="Output")
  plot!(t, iTerm, label="I Term", ls=:dash)

  plot(plotMain, plotSub, layout=(2, 1))
end