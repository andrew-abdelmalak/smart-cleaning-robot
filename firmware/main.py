import time
from machine import Pin, PWM, unique_id
import bluetooth
from micropython import const
import ubinascii

# ===========================================================
# BLUETOOTH CONSTANTS
# ===========================================================

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_UART_SERVICE_UUID = bluetooth.UUID(
    "6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_CHAR_UUID = (bluetooth.UUID(
    "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), 
    bluetooth.FLAG_NOTIFY)
_UART_RX_CHAR_UUID = (bluetooth.UUID(
    "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), 
    bluetooth.FLAG_WRITE)
_UART_SERVICE_DEFINITION = (_UART_SERVICE_UUID, 
    (_UART_TX_CHAR_UUID, _UART_RX_CHAR_UUID))

# Generate unique device name
uid_bytes = unique_id()
unique_suffix = ubinascii.hexlify(uid_bytes[-2:]).decode('utf-8')
_ADVERTISING_NAME = f"PicoRobot-{unique_suffix.upper()}"

# ===========================================================
# HARDWARE CONFIGURATION
# ===========================================================

# Motor pins
ENA_PIN, IN1_PIN, IN2_PIN = 6, 12, 13
ENB_PIN, IN3_PIN, IN4_PIN = 7, 14, 15

# Encoder pins
LEFT_ENCODER_PIN = 21
RIGHT_ENCODER_PIN = 20

# Peripheral pins
FAN_PIN, LED_PIN = 8, 18
TRIG_PIN, ECHO_PIN = 10, 11
BUTTON_PIN = 16

# ===========================================================
# ROBOT PARAMETERS
# ===========================================================

# Movement settings
MOVEMENT_PPS = 75.0
RIGHT_TRIM = 0.75

# Navigation
OBSTACLE_THRESHOLD_CM = 15
TURN_PULSES = 39
REVERSE_PULSES = 30

# Brake settings
BRAKE_DUTY_PERCENT = 50
BRAKE_DURATION_MS = 50

# PID gains
GAINS_STRAIGHT = {"P": 0.8, "I": 0.15, "D": 0.25}
GAINS_TURN_PROGRESS = {"P": 0.6, "I": 0.1, "D": 0.2}
GAINS_TURN_BALANCE = {"P": 1.5, "I": 0.05, "D": 0.3}
INTEGRAL_CLAMP_LIMIT = 15.0

# Safety timing
STALL_TIME_MS = 800
STALL_CHECK_INTERVAL_MS = 250
FAN_SPINUP_DELAY_MS = 500

# LED timing
LED_SLOW_FLASH_MS = 500
LED_RAPID_FLASH_MS = 100

# ===========================================================
# HARDWARE INITIALIZATION
# ===========================================================

# Motor PWM
ena, enb = PWM(Pin(ENA_PIN)), PWM(Pin(ENB_PIN))
ena.freq(1000)
enb.freq(1000)

# Motor direction
in1, in2 = Pin(IN1_PIN, Pin.OUT), Pin(IN2_PIN, Pin.OUT)
in3, in4 = Pin(IN3_PIN, Pin.OUT), Pin(IN4_PIN, Pin.OUT)

# Encoders
left_enc = Pin(LEFT_ENCODER_PIN, Pin.IN, Pin.PULL_UP)
right_enc = Pin(RIGHT_ENCODER_PIN, Pin.IN, Pin.PULL_UP)

# Peripherals
fan = Pin(FAN_PIN, Pin.OUT)
led = Pin(LED_PIN, Pin.OUT)
trig, echo = Pin(TRIG_PIN, Pin.OUT), Pin(ECHO_PIN, Pin.IN)
button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP)

# ===========================================================
# GLOBAL STATE
# ===========================================================

# Encoder counters
left_pulses, right_pulses = 0, 0

# PID states
straight_integral, straight_last_error = 0, 0
progress_integral, progress_last_error = 0, 0
balance_integral, balance_last_error = 0, 0

# Pivot tracking
pivot_start_L, pivot_start_R = 0, 0

# Operating modes
autonomous_mode = False
autonomous_state = "IDLE"
manual_command = None

# Communication
last_ble_command = ""
ble_connected = False

# Safety & feedback
forward_blocked = False
led_state = "OFF"
last_led_toggle = 0
horn_active = False

# Stall detection
stall_start_time = 0
last_check_left = 0
last_check_right = 0
next_stall_check = 0

# Enhanced safety
last_motor_time = 0
emergency_active = False

# ===========================================================
# ENCODER HANDLERS
# ===========================================================

def left_encoder_isr(pin):
    global left_pulses
    left_pulses += 1

def right_encoder_isr(pin):
    global right_pulses
    right_pulses += 1

left_enc.irq(trigger=Pin.IRQ_RISING, handler=left_encoder_isr)
right_enc.irq(trigger=Pin.IRQ_RISING, handler=right_encoder_isr)

# ===========================================================
# MOTOR CONTROL
# ===========================================================

def ff_pwm(pps_target: float, wheel: str) -> int:
    """Feed-forward PWM calculation"""
    if wheel == 'L':
        kS, kV = 27, 0.66
    else:
        kS, kV = 30, 0.61
    return min(95, int(kS + kV * pps_target))

def set_motor_pwm(left_pwm, right_pwm, direction="stop"):
    """Set motor PWM with direction control"""
    global last_motor_time
    
    if left_pwm > 0 or right_pwm > 0:
        last_motor_time = time.ticks_ms()
    
    # Swap for physical wiring
    phys_left_pwm = right_pwm
    phys_right_pwm = left_pwm
    
    # Direction control
    if direction == "forward":
        in3.high()
        in4.low()
        in1.high()
        in2.low()
    elif direction == "backward":
        in3.low()
        in4.high()
        in1.low()
        in2.high()
    elif direction == "pivot_left":
        in3.low()
        in4.high()
        in1.high()
        in2.low()
    elif direction == "pivot_right":
        in3.high()
        in4.low()
        in1.low()
        in2.high()
    else:
        phys_left_pwm, phys_right_pwm = 0, 0
    
    # Apply PWM
    ena.duty_u16(int(phys_left_pwm / 100 * 65535))
    enb.duty_u16(int(phys_right_pwm / 100 * 65535))

def straight_motion(direction, start_left=None, start_right=None):
    """Execute straight motion with PID"""
    global straight_integral, straight_last_error
    
    if start_left is None or start_right is None:
        # Simple feed-forward
        left_pps = MOVEMENT_PPS
        right_pps = MOVEMENT_PPS * RIGHT_TRIM
        
        left_pwm = ff_pwm(left_pps, 'L')
        right_pwm = ff_pwm(right_pps, 'R')
        
        set_motor_pwm(left_pwm, right_pwm, direction)
        return
    
    # Positional error
    error = ((left_pulses - start_left) - 
             (right_pulses - start_right))
    
    # Integral with clamp
    straight_integral += error
    if abs(straight_integral) > INTEGRAL_CLAMP_LIMIT:
        straight_integral = (INTEGRAL_CLAMP_LIMIT if 
                           straight_integral > 0 else 
                           -INTEGRAL_CLAMP_LIMIT)
    
    # Derivative
    derivative = error - straight_last_error
    straight_last_error = error
    
    # PID output
    adjustment = (GAINS_STRAIGHT["P"] * error +
                 GAINS_STRAIGHT["I"] * straight_integral +
                 GAINS_STRAIGHT["D"] * derivative)
    
    # Apply corrections
    left_pps = MOVEMENT_PPS - adjustment
    right_pps = (MOVEMENT_PPS * RIGHT_TRIM) + adjustment
    
    left_pwm = ff_pwm(abs(left_pps), 'L')
    right_pwm = ff_pwm(abs(right_pps), 'R')
    
    set_motor_pwm(left_pwm, right_pwm, direction)

def pivot_motion(dir, angle_pulses=None):
    """Execute pivot with dual PID"""
    global progress_integral, progress_last_error
    global balance_integral, balance_last_error
    global pivot_start_L, pivot_start_R
    
    direction = "pivot_left" if dir == "left" else "pivot_right"
    
    if angle_pulses is None:
        # Simple feed-forward
        left_pps = MOVEMENT_PPS
        right_pps = MOVEMENT_PPS * RIGHT_TRIM
        
        left_pwm = ff_pwm(left_pps, 'L')
        right_pwm = ff_pwm(right_pps, 'R')
        
        set_motor_pwm(left_pwm, right_pwm, direction)
        return
    
    # Calculate deltas
    left_delta = left_pulses - pivot_start_L
    right_delta = right_pulses - pivot_start_R
    
    # Progress PID
    total_progress = abs(left_delta) + abs(right_delta)
    progress_error = angle_pulses - total_progress
    progress_integral += progress_error
    if abs(progress_integral) > INTEGRAL_CLAMP_LIMIT:
        progress_integral = (INTEGRAL_CLAMP_LIMIT if 
                           progress_integral > 0 else 
                           -INTEGRAL_CLAMP_LIMIT)
    
    progress_derivative = progress_error - progress_last_error
    progress_last_error = progress_error
    
    progress_adj = (GAINS_TURN_PROGRESS["P"] * progress_error +
                   GAINS_TURN_PROGRESS["I"] * progress_integral +
                   GAINS_TURN_PROGRESS["D"] * progress_derivative)
    
    # Balance PID
    if dir == "left":
        balance_error = -left_delta - right_delta
    else:
        balance_error = left_delta + right_delta
    
    balance_integral += balance_error
    if abs(balance_integral) > INTEGRAL_CLAMP_LIMIT:
        balance_integral = (INTEGRAL_CLAMP_LIMIT if 
                          balance_integral > 0 else 
                          -INTEGRAL_CLAMP_LIMIT)
    
    balance_derivative = balance_error - balance_last_error
    balance_last_error = balance_error
    
    balance_adj = (GAINS_TURN_BALANCE["P"] * balance_error +
                  GAINS_TURN_BALANCE["I"] * balance_integral +
                  GAINS_TURN_BALANCE["D"] * balance_derivative)
    
    # Apply adjustments
    base_power = MOVEMENT_PPS + progress_adj
    left_pps = base_power + balance_adj
    right_pps = (base_power * RIGHT_TRIM) - balance_adj
    
    left_pwm = ff_pwm(abs(left_pps), 'L')
    right_pwm = ff_pwm(abs(right_pps), 'R')
    
    set_motor_pwm(left_pwm, right_pwm, direction)

def horn_buzz():
    """Motor buzz for horn"""
    buzz_duty = 10
    set_motor_pwm(buzz_duty, buzz_duty * RIGHT_TRIM, "forward")

def apply_brake():
    """Apply brake pulse"""
    set_motor_pwm(BRAKE_DUTY_PERCENT, 
                  BRAKE_DUTY_PERCENT * RIGHT_TRIM, "backward")
    time.sleep_ms(BRAKE_DURATION_MS)
    set_motor_pwm(0, 0, "stop")

# ===========================================================
# MOVEMENT STATE CONTROL
# ===========================================================

def stop_all_movement():
    """Emergency stop"""
    global manual_command, autonomous_state
    manual_command = None
    autonomous_state = "IDLE"
    set_motor_pwm(0, 0, "stop")

def is_full_stop():
    """Check if at full stop"""
    current = time.ticks_ms()
    motors_stopped = (ena.duty_u16() == 0 and enb.duty_u16() == 0)
    time_since = time.ticks_diff(current, last_motor_time)
    
    return motors_stopped and time_since >= 100

def emergency_stop():
    """Full emergency stop with state reset"""
    global emergency_active, autonomous_mode, autonomous_state
    global manual_command, forward_blocked, horn_active
    global straight_integral, straight_last_error
    global progress_integral, progress_last_error
    global balance_integral, balance_last_error
    
    emergency_active = True
    
    # Stop motion
    set_motor_pwm(0, 0, "stop")
    
    # Reset states
    autonomous_mode = False
    autonomous_state = "IDLE"
    manual_command = None
    horn_active = False
    
    # Reset PID controllers
    straight_integral = straight_last_error = 0
    progress_integral = progress_last_error = 0
    balance_integral = balance_last_error = 0
    
    # Turn off fan
    fan.off()
    
    # Clear emergency flag
    time.sleep_ms(50)
    emergency_active = False

# ===========================================================
# SENSORS & FEEDBACK
# ===========================================================

def get_distance_cm():
    """Ultrasonic distance with timeout"""
    trig.low()
    time.sleep_us(2)
    trig.high()
    time.sleep_us(10)
    trig.low()
    
    # Wait for echo start
    t_start = time.ticks_us()
    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), t_start) > 30000:
            return 999
    
    # Measure echo duration
    t1 = time.ticks_us()
    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), t1) > 30000:
            return 999
    
    t2 = time.ticks_us()
    return (time.ticks_diff(t2, t1) * 0.0343) / 2

def toggle_horn():
    """Toggle horn state"""
    global horn_active
    
    if not is_full_stop():
        return False
    
    horn_active = not horn_active
    if not horn_active:
        stop_all_movement()
    
    return True

def update_led():
    """Update LED based on state"""
    global last_led_toggle, led_state
    current = time.ticks_ms()
    
    # Determine LED state
    if forward_blocked or horn_active:
        new_state = "RAPID_FLASH"
    elif autonomous_mode:
        new_state = "SOLID_ON"
    elif ble_connected:
        new_state = "SLOW_FLASH"
    else:
        new_state = "OFF"
    
    # Reset on state change
    if new_state != led_state:
        last_led_toggle = current
        led_state = new_state
    
    # Apply pattern
    if led_state == "OFF":
        led.off()
    elif led_state == "SOLID_ON":
        led.on()
    elif led_state == "SLOW_FLASH":
        if time.ticks_diff(current, last_led_toggle) >= LED_SLOW_FLASH_MS:
            led.toggle()
            last_led_toggle = current
    elif led_state == "RAPID_FLASH":
        if time.ticks_diff(current, last_led_toggle) >= LED_RAPID_FLASH_MS:
            led.toggle()
            last_led_toggle = current

# ===========================================================
# BLUETOOTH UART CLASS
# ===========================================================

class BLEUART:
    """Bluetooth UART service"""
    
    def __init__(self, ble_obj, name=_ADVERTISING_NAME):
        self._ble = ble_obj
        self._ble.active(True)
        self._ble.irq(self._irq_handler)
        
        # Register service
        ((self._tx_handle, self._rx_handle),) = (
            self._ble.gatts_register_services(
                (_UART_SERVICE_DEFINITION,)))
        self._connections = set()
        
        # Prepare advertising
        self._adv_payload = (bytearray(b'\x02\x01\x06') +
                           bytes([len(name) + 1, 0x09]) +
                           name.encode())
        self._advertise()
    
    def _advertise(self):
        """Start advertising"""
        try:
            self._ble.gap_advertise(100_000,
                                   adv_data=self._adv_payload)
        except Exception:
            pass
    
    def _irq_handler(self, event, data):
        """Handle BLE events"""
        global ble_connected, last_ble_command
        
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            ble_connected = True
            
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.discard(conn_handle)
            if not self._connections:
                ble_connected = False
                emergency_stop()
                
        elif event == _IRQ_GATTS_WRITE:
            try:
                last_ble_command = (
                    self._ble.gatts_read(self._rx_handle)
                    .decode().strip().upper())
            except Exception:
                last_ble_command = ""

# ===========================================================
# MAIN PROGRAM LOOP
# ===========================================================

def main_program_loop():
    """Main control loop"""
    global autonomous_mode, autonomous_state, manual_command
    global last_ble_command, left_pulses, right_pulses
    global straight_integral, straight_last_error
    global progress_integral, progress_last_error
    global balance_integral, balance_last_error
    global pivot_start_L, pivot_start_R
    global forward_blocked, horn_active
    global stall_start_time, last_check_left
    global last_check_right, next_stall_check
    
    # Initialize
    left_start, right_start = 0, 0
    ble_uart = BLEUART(bluetooth.BLE())  # noqa: F841 — must be instantiated to register BLE IRQ
    
    while True:
        current_time = time.ticks_ms()
        
        # Emergency check
        if emergency_active:
            time.sleep_ms(20)
            continue
        
        # ===============================================
        # INPUT PROCESSING
        # ===============================================
        
        command = ""
        
        # Check BLE commands
        if ble_connected and last_ble_command:
            command = last_ble_command
            last_ble_command = ""
        
        # Check button
        if button.value() == 0:
            time.sleep_ms(50)
            if button.value() == 0:
                command = "CLEAN"
                while button.value() == 0:
                    time.sleep_ms(10)
        
        # ===============================================
        # COMMAND PROCESSING
        # ===============================================
        
        if command:
            if command == "CLEAN":
                if not is_full_stop():
                    emergency_stop()
                    continue
                else:
                    autonomous_mode = not autonomous_mode
                    if autonomous_mode:
                        fan.on()
                        time.sleep_ms(FAN_SPINUP_DELAY_MS)
                        autonomous_state = "START_FWD"
                    else:
                        fan.off()
                        
            elif command == "FAN":
                if is_full_stop():
                    fan.toggle()
                
            elif command == "HORN":
                if is_full_stop():
                    toggle_horn()
                
            # Obstacle clearing
            elif command == "FORWARD" and forward_blocked:
                current_dist = get_distance_cm()
                if current_dist >= OBSTACLE_THRESHOLD_CM:
                    forward_blocked = False
                    horn_active = False
                    continue
                elif horn_active:
                    horn_active = False
                    continue
                
            # Clear block when moving away
            elif (forward_blocked and 
                  command in ["BACKWARD", "LEFT", "RIGHT"]):
                forward_blocked = False
                horn_active = False
                
            # Manual commands with toggle
            if (not autonomous_mode and 
                command in ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]):
                if manual_command == command:
                    # Toggle: brake and stop
                    apply_brake()
                    manual_command = None
                else:
                    stop_all_movement()
                    manual_command = command
                    if command == "FORWARD":
                        left_start = left_pulses
                        right_start = right_pulses
                        straight_integral = straight_last_error = 0
                    elif command in ["LEFT", "RIGHT"]:
                        # Initialize pivot
                        pivot_start_L = left_pulses
                        pivot_start_R = right_pulses
                        balance_integral = balance_last_error = 0
        
        # ===============================================
        # MAIN LOOP EXECUTION (Priority Order)
        # ===============================================
        
        # Priority 1: Horn/Obstacle block
        if horn_active or forward_blocked:
            horn_buzz()
            update_led()
            time.sleep_ms(20)
            continue
        
        # Priority 2: Manual commands
        elif manual_command:
            if manual_command == "FORWARD":
                # Check obstacles
                if get_distance_cm() < OBSTACLE_THRESHOLD_CM:
                    forward_blocked = True
                    horn_active = True
                    manual_command = None
                    continue
                straight_motion("forward", left_start, right_start)
            elif manual_command == "BACKWARD":
                straight_motion("backward")
            elif manual_command == "LEFT":
                pivot_motion("left")
            elif manual_command == "RIGHT":
                pivot_motion("right")
        
        # Priority 3: Autonomous FSM
        elif autonomous_mode:
            
            if autonomous_state == "START_FWD":
                # Reset references
                left_start = left_pulses
                right_start = right_pulses
                straight_integral = straight_last_error = 0
                
                # Reset stall detection
                stall_start_time = 0
                last_check_left = left_pulses
                last_check_right = right_pulses
                next_stall_check = time.ticks_add(
                    current_time, STALL_CHECK_INTERVAL_MS)
                
                autonomous_state = "FWD"
            
            elif autonomous_state == "FWD":
                # Check obstacles
                if get_distance_cm() < OBSTACLE_THRESHOLD_CM:
                    apply_brake()
                    autonomous_state = "START_AVOID"
                    continue
                
                # Execute motion
                straight_motion("forward", left_start, right_start)
                
                # Stall detection
                stall_time_elapsed = time.ticks_diff(
                    current_time, next_stall_check)
                if stall_time_elapsed >= 0:
                    if (left_pulses == last_check_left or
                        right_pulses == last_check_right):
                        
                        if stall_start_time == 0:
                            stall_start_time = current_time
                        elif (time.ticks_diff(current_time, 
                                            stall_start_time) > 
                              STALL_TIME_MS):
                            print("STALL DETECTED! RECOVERING...")
                            # Recovery sequence
                            straight_motion("backward")
                            time.sleep_ms(700)
                            apply_brake()
                            
                            # Reset for stall turn
                            pivot_start_L = left_pulses
                            pivot_start_R = right_pulses
                            progress_integral = 0
                            progress_last_error = 0
                            balance_integral = balance_last_error = 0
                            autonomous_state = "STALL_TURN"
                            continue
                    else:
                        stall_start_time = 0
                    
                    # Update stall vars
                    last_check_left = left_pulses
                    last_check_right = right_pulses
                    next_stall_check = time.ticks_add(
                        current_time, STALL_CHECK_INTERVAL_MS)
            
            elif autonomous_state == "START_AVOID":
                # Initialize avoidance pivot
                pivot_start_L = left_pulses
                pivot_start_R = right_pulses
                progress_integral = progress_last_error = 0
                balance_integral = balance_last_error = 0
                autonomous_state = "AVOID"
            
            elif autonomous_state == "AVOID":
                # Execute avoidance turn
                current_pulses = ((left_pulses - pivot_start_L) + 
                                (right_pulses - pivot_start_R))
                if abs(current_pulses) >= TURN_PULSES:
                    apply_brake()
                    autonomous_state = "START_FWD"
                else:
                    pivot_motion("left", TURN_PULSES)
            
            elif autonomous_state == "STALL_TURN":
                # Execute stall recovery turn
                current_pulses = ((left_pulses - pivot_start_L) + 
                                (right_pulses - pivot_start_R))
                if abs(current_pulses) >= TURN_PULSES:
                    apply_brake()
                    autonomous_state = "START_FWD"
                else:
                    pivot_motion("left", TURN_PULSES)
        
        # Priority 4: Safety - ensure stopped when idle
        else:
            if ena.duty_u16() > 0 or enb.duty_u16() > 0:
                stop_all_movement()
        
        # Update interface
        update_led()
        time.sleep_ms(20)

# ===========================================================
# ENTRY POINT
# ===========================================================

if __name__ == "__main__":
    try:
        main_program_loop()
    except Exception as e:
        print(f"[FATAL] Exception: {e}")
        emergency_stop()
    finally:
        print("[SYSTEM] Program terminated.")
        emergency_stop()
        fan.off()
        led.off()
