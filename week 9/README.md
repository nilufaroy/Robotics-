# CS413 Robotics — Lab 9: Gyroscope parameters

Identify the parameters of the TRIK gyroscope (yaw axis), build a simulated gyro `gyro_sim` from them, and use them in Labs 10–12:

$$\omega_{meas}(t_k) = Q_{\Delta}\Big(\mathrm{sat}_{\pm\omega_{max}}\big[(1+s)\,\omega_{true}(t_k) + b_0 + d\,t_k + n_k\big]\Big), \quad n_k \sim \mathcal{N}(0,\sigma^2)$$

## Experiment: Option A (open loop, in-place rotation)

One program runs the whole sequence: `lab9_gyro_real.qrs` on the robot. The robot only logs, and everything is processed offline in the notebook.
Wheels: `u_R = u(t)` on M4/E4 (right), `u_L = -u(t)` on M3/E3 (left), with `u = sat±100(A·f(t))`.

| Phase | What the robot does | Duration | Used in |
|---|---|---|---|
| warm-up | still, not logged | 2 min | — |
| `static` | still, logged | 3 min | §1, §2, §4, §5 |
| `sin` | f = sin(2πt/4), A = 30 → 60 → 150 % (every 3 periods) | 36 s | §3, §6 |
| `rect` | square wave ±1, T = 4 s, A = 30 → 60 → 150 % | 36 s | §3, §6 |
| `rest` | still, logged | 3 s | final angles |

The robot sits on a rigid surface with the motors off during warm-up and static.


Log line every ~10 ms, written in blocks of 100 lines: `t_ms, t_us, gz_raw, gz_cal, encL, encR, u, phase`

| Column | Source | Unit |
|---|---|---|
| `t_ms` | `script.time()`, from the start of `static` | ms |
| `t_us` | `brick.gyroscope().read()[3]` | 1 unit = 256 µs (trikRuntime `TimeVal`), checked in the notebook |
| `gz_raw` | `brick.gyroscope().readRawData()[2]` | raw, uncalibrated |
| `gz_cal` | `brick.gyroscope().read()[2]` | mdeg/s |
| `encL`, `encR` | `brick.encoder("E3").read()`, `brick.encoder("E4").read()` | degrees (`Encoder::read()` converts ticks to degrees) |
| `u`, `phase` | motor command; phase (0 static, 1 sin, 2 rect, 3 rest) | %, — |

Reference heading from the wheels: θ_enc = r/B · (φ_R − φ_L), with r = 0.042 m and B = 0.194 m from Lab 5.

## Files

| File | Description |
|---|---|
| `lab9_gyro_real.qrs` | TRIK Studio block program for the real robot (Option A): warm-up, static run, rotation run, gyro + encoder logging in one loop. No processing on the robot. Upload and run it on the robot (the "Read raw gyro" block is JavaScript code). |
| `lab9_gyro_real.csv` | Raw log from the robot, all phases (run 1, used for the analysis) |
| `lab9_gyro_run2.csv` | Raw log of a second run, compared in §11 |
| `lab9_gyro.ipynb` | Colab notebook: units and sign, b₀(T), `calibrate_gyro()`, drift curves, saturation plot, resolution, noise histogram, θ_gyro vs θ_enc with e(t), scale-factor fit, `gyro_sim()` overlaid on the real log, parameter table, short analysis |

## Analysis (notebook sections)

| § | Parameter | Method |
|---|---|---|
| 0 | raw → deg/s factor F, sign | slope of `read()[2]` against `readRawData()[2]`. The sign comes from the correlation of the gyro with the encoder rate. |
| 1 | offset b₀ | mean of the first 30 s ± standard error, then b₀(T) for T = 1…30 s |
| 1 | `calibrate_gyro(log)` | first `T_CAL` s of the static log, spikes beyond 4 robust σ from the median removed, then the mean. `T_CAL` is the shortest T where ±2 SE fits within ±0.005 deg/s and b₀(T) stays flat afterwards. |
| 2 | drift d | line fit through the means of the 30-s windows. Heading is integrated with the trapezoid rule on the gyro timestamps: (a) raw rate, (b) rate − b₀ |
| 3 | range ±ω_max | flat top of the gyro against the encoder rate in the A = 150 % turns |
| 4 | resolution Δ | smallest step between distinct raw values while still, × F |
| 5 | noise σ | std of the static log minus b₀, histogram with a Gaussian fit |
| 6 | scale factor s | each turn from one stop to the next: Δθ_gyro against Δθ_enc, slope through the origin = 1 + s, CCW and CW separately. Saturated turns are excluded. s is also listed per amplitude and for sine vs square wave. |
| 7 | e(t) | θ_gyro, θ_enc and e = θ_gyro − θ_enc over the rotation run, the change of e in each part of the run |
| 8 | `gyro_sim(omega_true, t)` | the model above, fed with the encoder rate of the recorded run and plotted over the real gyro |

The parameter table (§9) and the short analysis (§10) are generated from the log by the notebook; §11 compares the two runs.

## Results

| Parameter | Symbol | Run 1 | Run 2 | Unit | Method |
|---|---|---|---|---|---|
| Offset (bias) | $b_0$ | −0.539 ± 0.004 | −0.653 ± 0.003 | deg/s | static mean, T = 30 s |
| Bias drift | $d$ | −0.007 | +0.011 | deg/s/min | 30-s window means |
| Heading drift, uncorrected | | −33.5 | −37.3 | deg/min | slope of θ(t) |
| Measurement range | ±$\omega_{max}$ | +286.5 / −286.7 | +287.3 / −287.6 | deg/s | saturation at A = 150 % |
| Resolution | $\Delta$ | 0.00875 | 0.00878 | deg/s | 1 raw unit × F |
| Noise std | $\sigma$ | 0.071 | 0.074 | deg/s | static, 3 min (one sample per ~73 ms) |
| Raw → deg/s factor | F | 0.00875 | 0.00878 | deg/s per raw unit | `read()[2]` vs `readRawData()[2]` |
| Scale-factor error CCW / CW | $s$ | −3.3 / −4.5 | −2.9 / −3.5 | % | gyro vs. encoder angle, unsaturated turns |
| Positive rotation direction | | CW | CW | — | gyro vs. encoder sign |

**Short analysis (run 1).**
- Without calibration the offset dominates at every time: 5.4° after 10 s, 32° after 1 min, 97° after 3 min.
- After `calibrate_gyro` (30 s average, spikes removed), what is left is 3.1° after 3 min still. The leftover offset dominates up to 1 min, and drift after that; noise stays below 0.3°.
- When turning, the scale factor costs about 16° per full turn, and every turn faster than ±287 deg/s loses about 100° to saturation, the largest error of all.
- Square-wave turns give a more negative s than sine turns: the wheels slip at hard reversals, so the encoders over-count.
- Run 2 repeats F, Δ, σ, ω_max, s and the sign. Only b₀ and d change, so `calibrate_gyro` has to run at every start.
