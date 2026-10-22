# Lab 5 – Motor parameter identification

Identification of the TRIK motor parameters $T_m$ and $k_e$ for both drive wheels from a PWM sweep (−100 % … +100 %, step 5 %), wheels spinning freely in the air. The results are in **[report.md](report.md)**.

## Files

| File | Description |
| --- | --- |
| `motor_identification.qrs` | TRIK Studio block program used to collect the data: for every PWM level it drives M3 and M4 for 2 s and records time and both encoders. |
| `motor_data.txt` | The recorded data, as the robot wrote it. One line per sample: `PWM_%  t_ms  E3_deg (left)  E4_deg (right)  battery_V`. |
| `identification.ipynb` | Analysis: loads the data, fits $\theta(t) = \omega_{ss}(t - T_m + T_m e^{-t/T_m})$ to every run, finds $k_e$ and $T_m$, simulates the motor with `odeint`. |
| `report.md` | Experiment, method, results and discussion. |
| `report_figs/omega_all_wheels.png` | ω_ss against voltage, both wheels, with the fitted $k_e$ lines. |
| `report_figs/step_response.png` | Measured vs simulated $\theta(t)$ at PWM = 100 %. |
| `report_figs/table.md` | $T_m$ and $\omega_{ss}$ for every PWM level, both wheels. |

## Results

| Wheel | $k_e$, V·s/rad | $T_m$, s | dead zone |
|---|---:|---:|---:|
| 1 – left (M3/E3) | 0.653 | 0.149 | none at ±5 % |
| 2 – right (M4/E4) | 0.705 | 0.126 | none at ±5 % |
