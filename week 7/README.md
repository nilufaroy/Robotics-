# CS413 Robotics — Lab 7: Go-to-goal

The last two digits of my student ID, **26**, driven as waypoints with the go-to-goal controller from the lecture, first in the TRIK Studio simulator and then on the real robot (same program, same gains).

## Waypoints (cm)

The robot starts at (0, 40) facing +x.

```
digit 2: (0,40) → (25,40) → (25,20) → (0,0) → (25,0)
gap:     (25,0) → (45,20)
digit 6: (45,20) → (70,20) → (70,0) → (45,0) → (45,40) → (70,40)
```

## Controller

| Quantity | In the program |
|---|---|
| Distance | `dist = sqrt(dx² + dy²)` |
| Heading error | `err = atan2(sin(θ_g − θ), cos(θ_g − θ))` (wrapped into (−π, π]) |
| Linear | `v = min(K_v·dist, v_max)`, kept ≥ `v_min`; `v = 0` when `abs(err) > 0.5` rad (turn in place first) |
| Angular | `u = K_θ · err` |
| Wheels | M4 (right) = `v + u`, M3 (left) = `v − u` |

Gains: `K_v = 3 %/cm`, `v_max = 60 %`, `v_min = 20 %`, `K_θ = 50 %/rad`, acceptance radius `ε = 3 cm`.
`r` and `B`: simulator 2.8 cm / 15.4 cm, real robot 4.2 cm / 19.4 cm (only these differ between the two programs).

## Files

| File | Description |
|---|---|
| `go_to_goal_sim.qrs` | TRIK Studio program for the simulator, writes `odometry_data_sim.txt` |
| `go_to_goal_real.qrs` | Same program for the real robot, writes `odometry_data_real.txt` |
| `odometry_data_sim.txt`, `odometry_data_real.txt` | Raw logs, one line per sample: `x,y,time` (cm, cm, ms) |
| `Lab7.ipynb` | Trajectory plot, waypoint-miss table, run times, tape check, conclusions |

## Results

| | sim | real |
|---|---:|---:|
| Largest waypoint miss (odometry) | 2.9 cm | 2.5 cm |
| Run time | 29.8 s | 13.8 s |

## Measured end point

| | x, cm | y, cm | distance to last waypoint (70, 40) |
|---|---:|---:|---:|
| Last line of `odometry_data_real.txt` | 68.2 | 38.3 | 2.5 cm |
| Tape | | | |

## Conclusions

See the last cell of `Lab7.ipynb`.