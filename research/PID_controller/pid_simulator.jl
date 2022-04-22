using Plots

include("pid_runner.jl")

# Parameters of PID
kp_1 = 150 / 100
ki_1 = 20 / 100
kd_1 = 80 / 100
pidMax_1 = 25
pidMin_1 = -pidMax_1

kp_2 = 250 / 100
ki_2 = 4 / 100
kd_2 = 5 / 100
pidMax_2 = 25
pidMin_2 = -pidMax_2

const count = 300
initInput = 25.0

# Generate setpoint data
function getSetPoint(t)
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
system(lastInput, feedback) = lastInput + feedback * 0.3 + 1

(setPoint_1, input_1, output_1, iTerm_1) = pid_run(kp_1, ki_1, kd_1, initInput, count, getSetPoint, system, max=pidMax_1, min=pidMin_1)

(setPoint_2, input_2, output_2, iTerm_2) = pid_run(kp_2, ki_2, kd_2, initInput, count, getSetPoint, system, max=pidMax_2, min=pidMin_2)

# Plot out
t = 1:count

plotMain = plot(t, input_1, label="Input-1")

# plot!(t, setPoint_2, label="Setpoint-2", linestyle=:dash)
plot!(t, input_2, label="Input-2")
plot!(t, setPoint_1, label="Setpoint-1", linestyle=:dash)

plotSub = plot(t, output_1, label="Output-1", ls=:dash)
plot!(t, iTerm_1, label="I Term-1")

plot!(t, output_2, label="Output-2", ls=:dash)
plot!(t, iTerm_2, label="I Term-1")

plot(plotMain, plotSub, layout=(2, 1))