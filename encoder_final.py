import pigpio
import time
import math
import sys


import select

# --- GPIO Pin Definitions (BCM numbering) ---
IN1_A, IN2_A, ENA   = 17, 27, 22  # Motor A
IN1_B, IN2_B, ENB   = 23, 24, 25  # Motor B
ENC_A_PIN, ENC_B_PIN = 18, 5      # Encoder channels

# --- Encoder & Wheel Parameters ---
GEAR_RATIO      = 600
LINES_PER_REV   = 11
EDGE_MULTIPLIER = 2
PULSES_PER_REV  = GEAR_RATIO * LINES_PER_REV * EDGE_MULTIPLIER  # 13200

WHEEL_RADIUS_M = 0.0325
WHEEL_CIRC_M   = 2 * math.pi * WHEEL_RADIUS_M

INITIAL_H = 0.00

# --- PID Setpoints & Tolerances ---
TARGET_HEIGHT_M  = 1.20   # upward target (m)
HEIGHT_TOLERANCE = 0.001

INITIAL_DUTY_RATIO = 0.2
PWM_FREQ = 15000
MAX_DUTY  = 255

# --- PID Gains ---
Kp_h, Ki_h, Kd_h = 250.0, 10.0, 150.0
Kp_t, Ki_t, Kd_t = 200.0, 5.0, 20.0

# --- State Variables ---
pulse_count_A = 0
pulse_count_B = 0
cum_revs_A    = 0.0
cum_revs_B    = 0.0


# after PID gain setup, before "initialize pigpio"
cum_revs_A = INITIAL_H / WHEEL_CIRC_M
cum_revs_B = INITIAL_H / WHEEL_CIRC_M

height_int_A  = height_prev_A = 0.0
height_int_B  = height_prev_B = 0.0
tilt_int      = 0.0
tilt_prev     = 0.0

last_time = time.perf_counter()

def encoder_cb_a(gpio, level, tick):
    global pulse_count_A
    if level >= 0:
        pulse_count_A += 1

def encoder_cb_b(gpio, level, tick):
    global pulse_count_B
    if level >= 0:
        pulse_count_B += 1

# --- pigpio init ---
pi = pigpio.pi()
if not pi.connected:
    print("ERROR: pigpio daemon not running")
    sys.exit(1)

# --- Motor pins ---
for pin in (IN1_A, IN2_A, ENA, IN1_B, IN2_B, ENB):
    pi.set_mode(pin, pigpio.OUTPUT)

# set forward for upward climb
pi.write(IN1_A, 0); pi.write(IN2_A, 1)
pi.write(IN1_B, 0); pi.write(IN2_B, 1)

pi.set_PWM_frequency(ENA, PWM_FREQ)
pi.set_PWM_frequency(ENB, PWM_FREQ)
pi.set_PWM_dutycycle(ENA, 0)
pi.set_PWM_dutycycle(ENB, 0)

# --- Encoder inputs ---
for pin in (ENC_A_PIN, ENC_B_PIN):
    pi.set_mode(pin, pigpio.INPUT)
    pi.set_pull_up_down(pin, pigpio.PUD_UP)

cbA = pi.callback(ENC_A_PIN, pigpio.EITHER_EDGE, encoder_cb_a)
cbB = pi.callback(ENC_B_PIN, pigpio.EITHER_EDGE, encoder_cb_b)

print(f"Climbing up to {TARGET_HEIGHT_M:.2f}m ")
print("Press Ctrl+C to abort")

base_duty = int(MAX_DUTY * INITIAL_DUTY_RATIO)
pi.set_PWM_dutycycle(ENA, base_duty)
pi.set_PWM_dutycycle(ENB, base_duty)

upward_done = False
paused = False
terminate = False

try:
    # ---- Upward climb ----
    while True:
        if terminate:
            break

        now = time.perf_counter()
        dt  = now - last_time
        last_time = now

        # check for pause/terminate
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            cmd = sys.stdin.readline().strip().lower()
            if cmd == 'q':
                if paused:
                    print("Exiting.")
                    terminate = True
                    break
                print("Paused. ")
                pi.set_PWM_dutycycle(ENA, 0)
                pi.set_PWM_dutycycle(ENB, 0)
                paused = True
                continue
            elif cmd == 'r' and paused:
                print("Resuming upward climb...")
                pi.set_PWM_dutycycle(ENA, base_duty)
                pi.set_PWM_dutycycle(ENB, base_duty)
                paused = False

        if paused:
            time.sleep(0.1)
            continue

        cntA, cntB = pulse_count_A, pulse_count_B
        pulse_count_A = pulse_count_B = 0

        revsA = cntA / PULSES_PER_REV
        revsB = cntB / PULSES_PER_REV

        cum_revs_A += revsA
        cum_revs_B += revsB

        heightA = cum_revs_A * WHEEL_CIRC_M
        heightB = cum_revs_B * WHEEL_CIRC_M
        avg_h   = 0.5 * (heightA + heightB)

        # Height PID A
        err_hA = TARGET_HEIGHT_M - heightA
        height_int_A += err_hA * dt
        der_hA = (err_hA - height_prev_A) / dt if dt>0 else 0.0
        height_prev_A = err_hA
        pid_hA = Kp_h*err_hA + Ki_h*height_int_A + Kd_h*der_hA

        # Height PID B
        err_hB = TARGET_HEIGHT_M - heightB
        height_int_B += err_hB * dt
        der_hB = (err_hB - height_prev_B) / dt if dt>0 else 0.0
        height_prev_B = err_hB
        pid_hB = Kp_h*err_hB + Ki_h*height_int_B + Kd_h*der_hB

        # Tilt PID
        err_t = heightB - heightA
        tilt_int += err_t * dt
        der_t = (err_t - tilt_prev) / dt if dt>0 else 0.0
        tilt_prev = err_t
        pid_t = Kp_t*err_t + Ki_t*tilt_int + Kd_t*der_t

        dutyA = base_duty + pid_hA + pid_t
        dutyB = base_duty + pid_hB - pid_t
        dutyA = max(0, min(MAX_DUTY, int(dutyA)))
        dutyB = max(0, min(MAX_DUTY, int(dutyB)))

        pi.set_PWM_dutycycle(ENA, dutyA)
        pi.set_PWM_dutycycle(ENB, dutyB)

        print(f"Ht:{avg_h:}m|A:{heightA:}B:{heightB:}|Duty A:{dutyA}B:{dutyB}")

        if abs(TARGET_HEIGHT_M - avg_h) <= HEIGHT_TOLERANCE:
            print("Target reached. Stopping motors.")
            pi.set_PWM_dutycycle(ENA, 0)
            pi.set_PWM_dutycycle(ENB, 0)
            upward_done = True
            print(f"final Ht:{avg_h}m|A:{heightA}B:{heightB:}|Duty A:{dutyA}B:{dutyB}")
            break

        time.sleep(0.1)

    if terminate:
        raise KeyboardInterrupt


    print("Pausing 60s before auto-descent...")
    time.sleep(60)






    # reverse motor direction
    pi.write(IN1_A, 1); pi.write(IN2_A, 0)
    pi.write(IN1_B, 1); pi.write(IN2_B, 0)

    # reset counters & integrators (keep tilt_int, tilt_prev)
    offsetA = heightA
    offsetB = heightB
    cum_revs_A = cum_revs_B = 0.0

    pulse_count_A = pulse_count_B = 0
    height_int_A = height_int_B = 0.0
    height_prev_A = height_prev_B = 0.0
    last_time = time.perf_counter()

    print("Starting downward climb...")

    # ---- Downward climb (target = 0) ----
    while True:
        if terminate:
            break

        now = time.perf_counter()
        dt = now - last_time
        last_time = now

        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            cmd = sys.stdin.readline().strip().lower()
            if cmd == 'q':
                if paused:
                    print("Exiting.")
                    terminate = True
                    break
                print("Paused. ")
                pi.set_PWM_dutycycle(ENA, 0)
                pi.set_PWM_dutycycle(ENB, 0)
                paused = True
                continue
            elif cmd == 'r' and paused:
                print("Resuming downward climb...")
                pi.set_PWM_dutycycle(ENA, base_duty)
                pi.set_PWM_dutycycle(ENB, base_duty)
                paused = False

        if paused:
            time.sleep(0.1)
            continue


        cntA, cntB = pulse_count_A, pulse_count_B
        pulse_count_A = pulse_count_B = 0

        revsA = cntA / PULSES_PER_REV
        revsB = cntB / PULSES_PER_REV

        cum_revs_A += revsA
        cum_revs_B += revsB

        heightA = offsetA - cum_revs_A * WHEEL_CIRC_M
        heightB = offsetB - cum_revs_B * WHEEL_CIRC_M
        avg_h   = 0.5 * (heightA + heightB)

        # Height PID A (down)
        err_hA = heightA
        height_int_A += err_hA * dt
        der_hA = (err_hA - height_prev_A) / dt if dt>0 else 0.0
        height_prev_A = err_hA
        pid_hA = Kp_h*err_hA + Ki_h*height_int_A + Kd_h*der_hA

        # Height PID B (down)
        err_hB = heightB
        height_int_B += err_hB * dt
        der_hB = (err_hB - height_prev_B) / dt if dt>0 else 0.0
        height_prev_B = err_hB
        pid_hB = Kp_h*err_hB + Ki_h*height_int_B + Kd_h*der_hB

        # Tilt PID (same)
        err_t = heightB - heightA
        tilt_int += err_t * dt
        der_t = (err_t - tilt_prev) / dt if dt>0 else 0.0
        tilt_prev = err_t
        pid_t = Kp_t*err_t + Ki_t*tilt_int + Kd_t*der_t

        dutyA = base_duty + pid_hA - pid_t
        dutyB = base_duty + pid_hB + pid_t
        dutyA = max(0, min(MAX_DUTY, int(dutyA)))
        dutyB = max(0, min(MAX_DUTY, int(dutyB)))



        pi.set_PWM_dutycycle(ENA, dutyA)
        pi.set_PWM_dutycycle(ENB, dutyB)

        print(f"Ht:{avg_h:}m|A:{heightA:}B:{heightB:}|Duty A:{dutyA}B:{dutyB}")

        if abs(avg_h - INITIAL_H) <= HEIGHT_TOLERANCE or avg_h < 0:
            print("Reached bottom. Stopping motors.")
            pi.set_PWM_dutycycle(ENA, 0)
            pi.set_PWM_dutycycle(ENB, 0)
            print(f"Ht:{avg_h:}m|A:{heightA:}B:{heightB:}|Duty A:{dutyA}B:{dutyB}")
            break

            time.sleep(0.1)
        # else:
        #     print("climbing not started, exiting")

finally:
    cbA.cancel()
    cbB.cancel()
    pi.stop()
    print("Cleanup complete, exiting.")
