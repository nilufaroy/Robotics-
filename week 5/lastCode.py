# Lab 5 - motor parameter identification, data collection on TRIK.
# Runs a PWM step on each drive wheel for every level -100..-5, 5..100 %,
# samples (time, encoder angle) into a Python list and writes the file
# only after the motor is switched off.
#
# Output: motor_data/wheel_<N>_v_<PWM>.txt   (one per wheel and level)
#         motor_data/wheel_<N>_battery.txt   (battery at start of series)
#
# CHECK BEFORE RUNNING: the ports below must match YOUR robot.
# Lift the robot so both wheels spin freely (or put it on a stand).

import os
import time

# (wheel_id, motor_port, encoder_port)  -- wheel ids: 1..4 left to right
WHEELS = [
    (1, "M3", "E3"),     # left drive wheel
    (2, "M4", "E4"),     # right drive wheel
]

RUN_TIME  = 2.0          # s of recording per PWM level
DT        = 0.005        # s, target sampling period
SETTLE    = 0.5          # s after encoder reset
REST_TIME = 1.0          # s, let the motor stop between runs
OUT_DIR   = "motor_data"

LEVELS = list(range(-100, 0, 5)) + list(range(5, 105, 5))


def read_battery():
    return brick.battery().readVoltage()


def show(text):
    brick.display().clear()
    brick.display().addLabel(text, 1, 1)
    brick.display().redraw()


def record_step(motor_port, encoder_port, power):
    """Apply a PWM step from rest, return [(t_ms, angle_deg), ...]."""
    motor = brick.motor(motor_port)
    encoder = brick.encoder(encoder_port)

    motor.powerOff()
    encoder.reset()
    time.sleep(SETTLE)

    samples = []
    t0 = time.time()
    motor.setPower(power)                      # the step: t = 0
    while True:
        t = time.time() - t0
        if t > RUN_TIME:
            break
        samples.append((int(t * 1000), encoder.read()))   # real time stamp
        time.sleep(DT)
    motor.powerOff()
    return samples


def save(wheel_id, power, battery, samples):
    name = "%s/wheel_%d_v_%d.txt" % (OUT_DIR, wheel_id, power)
    lines = ["# wheel_id: %d\n" % wheel_id,
             "# pwm_percent: %d\n" % power,
             "# battery_voltage: %s\n" % battery,
             "# format: time_ms angle_deg\n"]
    for t_ms, angle in samples:
        lines.append("%d %s\n" % (t_ms, angle))
    f = open(name, "w")
    f.write("".join(lines))
    f.close()


def run_wheel(wheel_id, motor_port, encoder_port):
    battery = read_battery()
    f = open("%s/wheel_%d_battery.txt" % (OUT_DIR, wheel_id), "w")
    f.write("battery_voltage: %s\n" % battery)
    f.close()
    print("Wheel %d (%s/%s), battery %s V" % (wheel_id, motor_port, encoder_port, battery))

    for power in LEVELS:
        show("W%d PWM %d %%" % (wheel_id, power))
        battery = read_battery()               # motor is off here -> no load sag
        samples = record_step(motor_port, encoder_port, power)
        save(wheel_id, power, battery, samples)   # file I/O only after motor off

        last = samples[-1][1] if samples else "none"
        print("W%d PWM %4d %%: %d samples, final angle %s deg"
              % (wheel_id, power, len(samples), last))
        time.sleep(REST_TIME)


def main():
    if not os.path.exists(OUT_DIR):
        os.makedirs(OUT_DIR)
    where = os.path.abspath(OUT_DIR)
    print("Saving data to: " + where)          # this is the folder to copy later
    for wheel_id, motor_port, encoder_port in WHEELS:
        run_wheel(wheel_id, motor_port, encoder_port)
    show("Done: " + where)
    print("Done. Files are in " + where)
    brick.playTone(1000, 300)                  # audible "done"
    brick.stop()


main()
