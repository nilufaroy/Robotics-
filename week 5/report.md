# Lab 5 – Motor parameter identification

## Experiment

The robot stood on a support so **both wheels spin freely in the air**. The block program `motor_identification.qrs` runs both motors (M3 = left, M4 = right) with the same PWM, from −100 % to +100 % in steps of 5 %. Each level lasts 2 s. During that time it records the time and both encoder angles (E3, E4); after the motors stop it writes them to `motor_data.txt`.

The applied voltage is U = PWM/100 · U_battery. The block program cannot read the battery, so I used the value shown on the robot, **11.1 V**.

## Method

Motor model: T_m · dω/dt + ω = U / k_e. For a voltage step the angle is θ(t) = ω_ss · (t − T_m + T_m · e^(−t/T_m)), with ω_ss = U / k_e.

1. For every PWM level and wheel, `curve_fit` finds ω_ss and T_m.
2. k_e = slope of ω_ss = U/k_e through zero, using |PWM| ≥ 30 %.
3. T_m = median over the same levels.
4. The model with these numbers is solved with `odeint` and compared with the measurement.

Code: [identification.ipynb](identification.ipynb).

## Results

| Wheel | k_e, V·s/rad | T_m, s |
|---|---:|---:|
| 1 – left (M3/E3) | 0.653 | 0.149 |
| 2 – right (M4/E4) | 0.705 | 0.126 |

### T_m and ω_ss for every PWM level

| PWM, % | U, V | ω_ss wheel 1, rad/s | T_m wheel 1, s | ω_ss wheel 2, rad/s | T_m wheel 2, s |
|---:|---:|---:|---:|---:|---:|
| -100 | -11.10 | -14.84 | 0.058 | -14.03 | 0.064 |
| -95 | -10.54 | -14.46 | 0.067 | -13.58 | 0.060 |
| -90 | -9.99 | -13.81 | 0.085 | -12.81 | 0.078 |
| -85 | -9.43 | -13.19 | 0.117 | -12.13 | 0.090 |
| -80 | -8.88 | -12.91 | 0.117 | -11.80 | 0.103 |
| -75 | -8.32 | -12.58 | 0.122 | -11.49 | 0.103 |
| -70 | -7.77 | -11.93 | 0.165 | -10.72 | 0.135 |
| -65 | -7.21 | -11.15 | 0.177 | -9.94 | 0.169 |
| -60 | -6.66 | -10.98 | 0.204 | -9.76 | 0.181 |
| -55 | -6.11 | -10.43 | 0.224 | -9.22 | 0.197 |
| -50 | -5.55 | -10.17 | 0.239 | -8.85 | 0.112 |
| -45 | -5.00 | -9.81 | 0.228 | -8.52 | 0.209 |
| -40 | -4.44 | -9.45 | 0.258 | -8.16 | 0.213 |
| -35 | -3.88 | -9.02 | 0.282 | -7.67 | 0.212 |
| -30 | -3.33 | -7.85 | 0.120 | -6.76 | 0.255 |
| -25 | -2.77 | -7.21 | 0.109 | -6.14 | 0.062 |
| -20 | -2.22 | -6.64 | 0.355 | -5.51 | 0.279 |
| -15 | -1.66 | -5.78 | 0.371 | -4.97 | 0.039 |
| -10 | -1.11 | -4.76 | 0.375 | -3.75 | 0.295 |
| -5 | -0.56 | -2.53 | 0.662 | -1.70 | 0.345 |
| 0 | 0.00 | 0.00 | — | 0.00 | — |
| 5 | 0.56 | 2.35 | 0.503 | 1.67 | 0.393 |
| 10 | 1.11 | 4.88 | 0.445 | 3.86 | 0.309 |
| 15 | 1.66 | 5.76 | 0.353 | 4.85 | 0.317 |
| 20 | 2.22 | 6.78 | 0.363 | 5.60 | 0.039 |
| 25 | 2.77 | 7.47 | 0.338 | 6.32 | 0.289 |
| 30 | 3.33 | 8.00 | 0.273 | 6.82 | 0.213 |
| 35 | 3.88 | 9.13 | 0.274 | 7.85 | 0.217 |
| 40 | 4.44 | 9.47 | 0.244 | 8.38 | 0.227 |
| 45 | 5.00 | 9.82 | 0.132 | 8.80 | 0.100 |
| 50 | 5.55 | 10.30 | 0.224 | 9.23 | 0.210 |
| 55 | 6.11 | 10.58 | 0.215 | 9.54 | 0.191 |
| 60 | 6.66 | 11.19 | 0.210 | 10.17 | 0.182 |
| 65 | 7.21 | 11.39 | 0.190 | 10.42 | 0.160 |
| 70 | 7.77 | 12.05 | 0.117 | 11.14 | 0.098 |
| 75 | 8.32 | 12.72 | 0.132 | 11.98 | 0.116 |
| 80 | 8.88 | 13.14 | 0.108 | 12.39 | 0.107 |
| 85 | 9.43 | 13.45 | 0.111 | 12.87 | 0.104 |
| 90 | 9.99 | 14.06 | 0.087 | 13.61 | 0.077 |
| 95 | 10.54 | 14.78 | 0.071 | 14.53 | 0.065 |
| 100 | 11.10 | 15.19 | 0.058 | 15.02 | 0.055 |

### ω_ss vs voltage

![omega vs U](report_figs/omega_all_wheels.png)

### Step response at 100 %, measured vs simulated

![step response](report_figs/step_response.png)

## Discussion

The linear model fails at low voltage: the measured speed rises much faster than the straight line ω = U/k_e, so the real curve is bent, not straight. With the wheels in the air there is no dead zone — both wheels already turn at ±5 %. T_m is not constant either: it is about 0.06 s at full power but 0.3–0.5 s at low PWM. The two wheels are different: at the same PWM wheel 1 is 1 % faster at 100 %, 12 % faster at 50 % and 26 % faster at 10 %. So next week, if both motors get the same power the robot will not drive straight, and odometry has to use each wheel's own encoder instead of assuming both wheels turn the same.
