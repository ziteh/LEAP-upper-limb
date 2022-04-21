include("pid.jl")
using Plots

function sp(np)
  if np > 70
    return -5
  elseif np > 20
    return 70
  elseif np > 10
    return 25
  else
    return 25
  end
end

t = 1:100
setpoint = Array{Int64}(undef, 100)
value = Array{Float64}(undef, 100)
it = Array{Float64}(undef, 100)
pidout = Array{Float64}(undef, 100)

kp = 150 / 100
ki = 20 / 100
kd = 80 / 100

int = 25
lsum = 0
lin = int
for i in t
  nsp = sp(i)
  setpoint[i] = nsp
  (out, ls, li) = pid_compute(nsp, int, kp, ki, kd, lsum, lin, 15, -15)
  global lsum = ls
  global it[i] = ls
  global lin = li

  global pidout[i] = out
  global int += out * 0.5 - 1

  global value[i] = int

end

plot(t, setpoint, label="Setpoint")
plot!(t, value, label="Value")
plot!(t, it, label="I")
plot!(t, pidout, label="Out")
