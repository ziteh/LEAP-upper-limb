# PID Simulator

A simple PID controller simulator.

## Usage

1. Open a new julia REPL.
2. Change the working dircetory to this folder:
```julia
julia> cd("PATH_TO_THIS_FOLDER")
```

3. Import `PidSimulator` module:
```julia
julia> import PidSimulator
```

### Simulator

Run `run()` function:
```julia
julia> PidSimulator.run()
```

julia REPL will return some info, like this:
```julia
--- pid_simulator ---
1st config:
kp: 1.500000, ki: 0.200000, Kd: 0.800000, max: 25.000000, min: -25.000000

2nd config:
kp: 2.500000, ki: 0.040000, Kd: 0.050000, max: 25.000000, min: -25.000000
```

### Plot

Run `plot()` function with specified `kp=1.5`, `ki=0.2` and `kd=0.3`:
```julia
julia> PidSimulator.plot(1.5, 0.2, 0.3)
```

Run `plot()` function with specified `kp`, `ki`, `kd` and `max=25`, `min=-25`:
```julia
julia> PidSimulator.plot(1.5, 0.2, 0.3, 25, -25)
```

Run `plot()` function with specified `kp`, `ki`, `kd`, `max`, `min` and `systemModel`:
```julia
julia> PidSimulator.plot(1.5, 0.2, 0.3, 25, -25, systemModel=(i,b)->i+b*0.7+5)
```

julia REPL will return some info, like this:
```julia
kp: 1.500000, ki: 0.200000, Kd: 0.300000, max: 25.000000, min: -25.000000
```

## Environment

- julia `v1.6.6 (LTS)` 64-bit
  - Plots `v1.27.6`