# Lab 5 – Motor parameter identification

Identification of the TRIK RK370 motor parameters $T_m$ and $k_e$ from a PWM sweep (−100 % … +100 %, step 5 %) on both drive wheels, wheels spinning freely in the air. The results are in **[report.md](report.md)**.

## Files

| File | Description |
| --- | --- |
| `motor_identification.qrs` | TRIK Studio block program used to collect the data. For every PWM level it clears E3/E4, drives M3 and M4 for 2 s, keeps the samples in memory and writes them to the file once the motors are stopped. |
| `motor_data.txt` | The recorded data, exactly as the robot wrote it. One line per sample: `PWM_%  t_ms  E3_deg (wheel 1, left)  E4_deg (wheel 2, right)  battery_V`. The block language cannot read the battery, so the last column is `0`; the voltage read on the robot (11.1 V) is set in the notebook. |
| `identification.ipynb` | Step-by-step analysis notebook: loads the data, fits $\theta(t) = \omega_{ss}(t - T_m + T_m e^{-t/T_m})$ to every run, computes $k_e$, simulates the motor with `odeint`, compares the two wheels. Writes everything in `report_figs/`. |
| `lastCode.py` | Python version of the same experiment (wheels one after the other, battery read automatically). Not used for this data set. |
| `identify.py` | Command-line version of the analysis (`python3 identify.py --data motor_data.txt --battery 11.1`), gives the same numbers as the notebook. |
| `report.md` | Experiment, method, results and discussion. |
| `report_figs/omega_all_wheels.png` | ω_ss against applied voltage, both wheels, with the fitted $k_e$ lines and the dead-zone band. |
| `report_figs/T_m.png` | $T_m$ for every run against applied voltage, both wheels. |
| `report_figs/step_response.png` | Measured vs simulated $\theta(t)$ at PWM = 100 % and 50 %. |
| `report_figs/table.md` | $T_m$ and $\omega_{ss}$ for every PWM level, both wheels (included in `report.md`). |

## Results

| Wheel | $k_e$, V·s/rad | $T_m$ (median, \|PWM\| ≥ 30 %), s | dead zone |
|---|---:|---:|---:|
| 1 – left (M3/E3) | 0.653 | 0.149 | none at ±5 % |
| 2 – right (M4/E4) | 0.705 | 0.126 | none at ±5 % |

## Reproduce

```sh
pip install numpy scipy matplotlib jupyter
jupyter notebook identification.ipynb      # or open it in Colab and upload motor_data.txt
```
