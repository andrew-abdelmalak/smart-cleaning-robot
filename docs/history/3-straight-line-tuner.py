# Straight-Line Tuner with Updated Motor Trim
#
# Objective: Test the PID controller with an updated RIGHT_MOTOR_TRIM
# value to better compensate for the physical motor imbalance.

import time
from machine import Pin, PWM

# =====================================================================================
# CALIBRATION & MOVEMENT CONSTANTS
# =====================================================================================
TARGET_PULSES_STRAIGHT = 59  # Target for 60cm drive

# =====================================================================================
# CONTROL, TUNING, AND PID GAINS
# =====================================================================================
BASE_SPEED_PERCENT = 70.0
# UPDATED: Changed from 0.8 to 0.75 to better match observed motor performance
RIGHT_MOTOR_TRIM = 0.75
BRAKE_POWER_PERCENT = 30.0
BRAKE_DURATION_MS = 50

INTEGRAL_CLAMP_LIMIT = 15.0

GAINS_STRAIGHT = {"P": 0.8, "I": 0.15, "D": 0.25}

# =====================================================================================
# HARDWARE PIN DEFINITIONS
# =====================================================================================
# Motor Control Pins (Software Left -> Physical Right, Software Right -> Physical Left)
ENA_PIN, IN1_PIN, IN2_PIN = 6, 12, 13
ENB_PIN, IN3_PIN, IN4_PIN = 7, 14, 15

# Encoder Input Pins
PHYSICAL_LEFT_ENCODER_PIN = 21
PHYSICAL_RIGHT_ENCODER_PIN = 20

# Button Pin
BUTTON_PIN_NUMBER = 16

# =====================================================================================
# HARDWARE INITIALIZATION
# =====================================================================================
ena, enb = PWM(Pin(ENA_PIN)), PWM(Pin(ENB_PIN))
ena.freq(1000); enb.freq(1000)

in1, in2 = Pin(IN1_PIN, Pin.OUT), Pin(IN2_PIN, Pin.OUT)
in3, in4 = Pin(IN3_PIN, Pin.OUT), Pin(IN4_PIN, Pin.OUT)

left_enc_pin = Pin(PHYSICAL_LEFT_ENCODER_PIN, Pin.IN, Pin.PULL_UP)
right_enc_pin = Pin(PHYSICAL_RIGHT_ENCODER_PIN, Pin.IN, Pin.PULL_UP)
start_button = Pin(BUTTON_PIN_NUMBER, Pin.IN, Pin.PULL_UP)

# =====================================================================================
# GLOBAL STATE VARIABLES
# =====================================================================================
left_wheel_pulses = 0
right_wheel_pulses = 0
straight_integral, straight_last_error = 0, 0

# =====================================================================================
# ENCODER INTERRUPT HANDLERS
# =====================================================================================
def left_encoder_callback(pin):
    global left_wheel_pulses; left_wheel_pulses += 1
def right_encoder_callback(pin):
    global right_wheel_pulses; right_wheel_pulses += 1

left_enc_pin.irq(trigger=Pin.IRQ_RISING, handler=left_encoder_callback)
right_enc_pin.irq(trigger=Pin.IRQ_RISING, handler=right_encoder_callback)

# =====================================================================================
# CORE MOTOR CONTROL & BRAKING
# =====================================================================================
def set_physical_motor_speeds(p_left_pwr, p_right_pwr, direction="stop"):
    # Correct for physical wiring swap: software commands are reversed to match motors
    s_left_pwr = p_right_pwr
    s_right_pwr = p_left_pwr

    if direction == "forward": in3.high(); in4.low(); in1.high(); in2.low()
    elif direction == "backward": in3.low(); in4.high(); in1.low(); in2.high()
    else: s_left_pwr, s_right_pwr = 0, 0

    l_duty = int(max(0, min(100, s_left_pwr)) / 100 * 65535)
    r_duty = int(max(0, min(100, s_right_pwr)) / 100 * 65535)
    ena.duty_u16(l_duty); enb.duty_u16(r_duty)

def apply_brake():
    print(f"  [BRAKE] Applying brake...")
    set_physical_motor_speeds(BRAKE_POWER_PERCENT, BRAKE_POWER_PERCENT, "backward")
    time.sleep_ms(BRAKE_DURATION_MS)
    set_physical_motor_speeds(0, 0, "stop")
    print("  [BRAKE] Brake complete.")

def stop_all_movement():
    set_physical_motor_speeds(0, 0, "stop")

# =====================================================================================
# PID CONTROLLER FUNCTION WITH INTEGRAL CLAMPING
# =====================================================================================
def positional_pid_straight(start_L, start_R):
    global straight_integral, straight_last_error
    
    error = (left_wheel_pulses - start_L) - (right_wheel_pulses - start_R)
    straight_integral += error
    
    # --- INTEGRAL CLAMPING ---
    if abs(straight_integral) > INTEGRAL_CLAMP_LIMIT:
        straight_integral = INTEGRAL_CLAMP_LIMIT if straight_integral > 0 else -INTEGRAL_CLAMP_LIMIT

    derivative = error - straight_last_error; straight_last_error = error
    adjustment = (GAINS_STRAIGHT["P"] * error) + (GAINS_STRAIGHT["I"] * straight_integral) + (GAINS_STRAIGHT["D"] * derivative)
    
    p_left = BASE_SPEED_PERCENT - adjustment
    # The trim is now applied here to adjust the baseline power
    p_right = (BASE_SPEED_PERCENT * RIGHT_MOTOR_TRIM) + adjustment
    
    print(f"[DRIVE] L:{left_wheel_pulses - start_L:<3} R:{right_wheel_pulses - start_R:<3} | Err:{error:<3} | Int:{straight_integral:<5.1f} | Adj:{adjustment:<5.1f} | L(pwr):{p_left:<5.1f} R(pwr):{p_right:<5.1f}")
    
    set_physical_motor_speeds(p_left, p_right, "forward")

# =====================================================================================
# MAIN PROGRAM
# =====================================================================================
def main():
    print("--- Straight-Line Tuner with Updated Motor Trim ---")
    print("Press button to start a 60cm straight-line test.")
    while start_button.value() == 1: time.sleep_ms(10)
    while start_button.value() == 0: time.sleep_ms(10)

    print("\n--- TEST: FORWARD (60cm) ---")
    
    global left_wheel_pulses, right_wheel_pulses
    global straight_integral, straight_last_error
    left_wheel_pulses, right_wheel_pulses = 0, 0
    straight_integral, straight_last_error = 0, 0
    
    left_start, right_start = 0, 0

    # CORRECTED: Changed 'start_start' to 'left_start'
    while ((left_wheel_pulses - left_start) + (right_wheel_pulses - right_start)) / 2 < TARGET_PULSES_STRAIGHT:
        positional_pid_straight(left_start, right_start)
        time.sleep_ms(20)
    
    apply_brake()

    print("\n--- TEST COMPLETE ---")

if __name__ == "__main__":
    try:
        main()
    except Exception as e:
        print(f"[FATAL] Exception occurred: {e}")
    finally:
        print("[SYSTEM] Program terminated - stopping motors.")
        stop_all_movement()

