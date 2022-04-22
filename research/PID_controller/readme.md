# PID Controller

A simple PID controller simulator.

## Usage

### Simulator

Run `pid_simulator.jl` from julia REPL:
```julia
julia> include("PATH_TO_THIS_FOLDER\\pid_simulator.jl")
```

### Plot

Include `pid_plot.jl` first from julia REPL:
```julia
julia> include("PATH_TO_THIS_FOLDER\\pid_plot.jl")
```

Run `pidPlot()` function with `kp=1.5`, `ki=0.2` and `kd=0.3`:
```julia
julia> pidPlot(1.5, 0.2, 0.3)
```

Run `pidPlot()` function with `kp`, `ki`, `kd` and `max=25`, `min=-25`:
```julia
julia> pidPlot(1.5, 0.2, 0.3, 25, -25)
```

Run `pidPlot()` function with `kp`, `ki`, `kd`, `max`, `min` and specified `systemModel`:
```julia
julia> pidPlot(1.5, 0.2, 0.3, 25, -25, systemModel=(i,b)->i+b*0.7+1)
```

## Environment

- julia `v1.6.6 (LTS)` x64
  - Plots `v1.27.5`