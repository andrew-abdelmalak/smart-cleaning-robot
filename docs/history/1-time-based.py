# --- Standard Library Imports ---
import bluetooth
from micropython import const
from machine import Pin, PWM, unique_id
import time
import ubinascii # For unique ID formatting

# --- BLE Configuration ---
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_CHAR_UUID = (bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"), bluetooth.FLAG_NOTIFY)
_UART_RX_CHAR_UUID = (bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"), bluetooth.FLAG_WRITE)
_UART_SERVICE_DEFINITION = (_UART_SERVICE_UUID, (_UART_TX_CHAR_UUID, _UART_RX_CHAR_UUID))

uid_bytes = unique_id()
unique_id_suffix = ubinascii.hexlify(uid_bytes[-2:]).decode('utf-8').upper()
_ADVERTISING_NAME = f"PicoRobot-{unique_id_suffix}"

# --- Pin Definitions ---
ENA_PIN = 6; ENA = PWM(Pin(ENA_PIN), freq=1000) # Left Motor PWM
IN1_PIN = 12; IN1 = Pin(IN1_PIN, Pin.OUT)       # Left Motor Direction 1
IN2_PIN = 13; IN2 = Pin(IN2_PIN, Pin.OUT)       # Left Motor Direction 2
ENB_PIN = 7; ENB = PWM(Pin(ENB_PIN), freq=1000) # Right Motor PWM
IN3_PIN = 14; IN3 = Pin(IN3_PIN, Pin.OUT)       # Right Motor Direction 1
IN4_PIN = 15; IN4 = Pin(IN4_PIN, Pin.OUT)       # Right Motor Direction 2
FAN_PIN = 8; FAN = Pin(FAN_PIN, Pin.OUT)
TRIG_PIN = 10; TRIG = Pin(TRIG_PIN, Pin.OUT)
ECHO_PIN = 11; ECHO = Pin(ECHO_PIN, Pin.IN)
BUTTON_PIN_NUMBER = 16; BTN_START_STOP = Pin(BUTTON_PIN_NUMBER, Pin.IN, Pin.PULL_UP)
LED_WORK_PIN = 18; LED_WORK = Pin(LED_WORK_PIN, Pin.OUT)

# --- Parameters for Autonomous Mode ---
AUTONOMOUS_FORWARD_SPEED_L = 65      # Left motor speed for autonomous forward
AUTONOMOUS_FORWARD_SPEED_R = 67.1      # Right motor speed for autonomous forward

# --- NEW: Specific Turn Speeds for Autonomous Mode ---
# Adjust these "factors" to fine-tune turning behavior
AUTONOMOUS_LEFT_TURN_MOTOR_L_SPEED = 65  # Speed of LEFT motor during a LEFT turn (e.g., backward or slower forward)
AUTONOMOUS_LEFT_TURN_MOTOR_R_SPEED = 67.025   # Speed of RIGHT motor during a LEFT turn (e.g., forward)

AUTONOMOUS_RIGHT_TURN_MOTOR_L_SPEED = 65  # Speed of LEFT motor during a RIGHT turn (e.g., forward)
AUTONOMOUS_RIGHT_TURN_MOTOR_R_SPEED = 67.025 # Speed of RIGHT motor during a RIGHT turn (e.g., backward or slower forward)
# --- End of New Turn Speeds ---

AUTONOMOUS_SHORT_MOVE_SPEED_L = 65 # Left motor speed for autonomous short forward moves
AUTONOMOUS_SHORT_MOVE_SPEED_R = 67.025  # Right motor speed for autonomous short forward moves
AUTONOMOUS_OBSTACLE_THRESHOLD_CM = 16

# New delay parameters as per user request
AUTONOMOUS_INITIAL_PAUSE_S = 1.5         # Pause after starting autonomous mode, before first move
AUTONOMOUS_OBSTACLE_DETECT_PAUSE_S = 1.5 # Pause after detecting an obstacle
AUTONOMOUS_POST_TURN_1_PAUSE_S = 1   # Pause after the first turn
AUTONOMOUS_POST_SHIFT_PAUSE_S = 1    # Pause after the short forward shift
AUTONOMOUS_POST_TURN_2_PAUSE_S = 1   # Pause after the second turn

TURN_DURATION_S = 1.25
SHIFT_MOVE_DURATION_S = 0.4

# --- Parameters for Manual BLE Mode ---
MANUAL_FORWARD_SPEED_L = 65         # Left motor speed for manual forward/backward
MANUAL_FORWARD_SPEED_R = 67.15        # Right motor speed for manual forward/backward
MANUAL_TURN_SPEED_L = 65          # Left motor speed for manual turning
MANUAL_TURN_SPEED_R = 67.15          # Right motor speed for manual turning
MOTOR_BUZZ_SPEED_L = 16.25             # Left motor low speed for Horn/Obstacle Assist Buzz
MOTOR_BUZZ_SPEED_R = 16.7875             # Right motor low speed for Horn/Obstacle Assist Buzz
MANUAL_OBSTACLE_THRESHOLD_CM = 15     # Closer threshold for manual mode safety assist

# --- Loop Delays & Timeouts ---
MAIN_LOOP_POLL_DELAY_MS = 20
AUTONOMOUS_LOGIC_INTERVAL_MS = 200 # This interval should be small enough for responsive state changes

# --- LED Control Parameters ---
LED_SLOW_FLASH_INTERVAL_MS = 500
LED_RAPID_FLASH_INTERVAL_MS = 100

# --- Global State Variables ---
robot_active_target_state = False
last_physical_button_action_time_ms = 0
BUTTON_DEBOUNCE_PERIOD_MS = 300

current_autonomous_action = "IDLE" # Possible states: IDLE, INITIAL_PAUSE, MOVING_FORWARD, OBSTACLE_PAUSING, TURNING_STEP_1, POST_TURN_1_PAUSE, SHIFTING_FORWARD, POST_SHIFT_PAUSE, TURNING_STEP_2, POST_TURN_2_PAUSE
action_end_time_ms = 0
perform_left_turn_sequence_next = True
next_autonomous_logic_run_time_ms = 0

ble_uart_instance = None
ble_is_connected = False
last_ble_command_str = ""
last_ble_command_time_ms = 0
active_ble_movement_command = None

manual_fan_is_on = False

current_led_state = "OFF"
last_led_toggle_time_ms = 0

# --- Utility Functions (set_speed_pwm, stop_motors_action, etc.) ---
def set_speed_pwm(pwm_motor_obj, speed_percent):
    """Sets the speed of a single motor PWM channel."""
    if speed_percent < 0: speed_percent = 0
    if speed_percent > 100: speed_percent = 100
    duty_u16_value = int((speed_percent / 100) * 65535)
    try:
        if pwm_motor_obj.freq() == 0: pwm_motor_obj.init(freq=1000) # Re-init if previously de-init
        pwm_motor_obj.duty_u16(duty_u16_value)
    except Exception as e: print(f"PWM_Err for {pwm_motor_obj}: {e}")


def stop_motors_action():
    """Stops both motors and clears active BLE movement command."""
    global active_ble_movement_command
    print("MOTORS: Stop action")
    IN1.low(); IN2.low() # Stop Left Motor
    IN3.low(); IN4.low() # Stop Right Motor
    set_speed_pwm(ENA, 0) # Set Left Motor speed to 0
    set_speed_pwm(ENB, 0) # Set Right Motor speed to 0
    active_ble_movement_command = None # Clear BLE command if motors are stopped for any reason


def start_move_forward_action(speed_percent_left, speed_percent_right):
    """Starts moving both motors forward with individual speeds."""
    print(f"MOTORS: Start Moving forward at L:{speed_percent_left}%, R:{speed_percent_right}%")
    set_speed_pwm(ENA, speed_percent_left)
    set_speed_pwm(ENB, speed_percent_right)
    IN1.high(); IN2.low() # Left Motor Forward
    IN3.high(); IN4.low() # Right Motor Forward


def start_move_backward_action(speed_percent_left, speed_percent_right):
    """Starts moving both motors backward with individual speeds."""
    print(f"MOTORS: Start Moving backward at L:{speed_percent_left}%, R:{speed_percent_right}%")
    set_speed_pwm(ENA, speed_percent_left)
    set_speed_pwm(ENB, speed_percent_right)
    IN1.low(); IN2.high() # Left Motor Backward
    IN3.low(); IN4.high() # Right Motor Backward


def start_turn_left_action(speed_percent_left, speed_percent_right):
    """Starts turning left (left motor backward/slower, right motor forward/faster)."""
    print(f"MOTORS: Start Turning left (actual) at L:{speed_percent_left}%, R:{speed_percent_right}%")
    set_speed_pwm(ENA, speed_percent_left)
    set_speed_pwm(ENB, speed_percent_right)
    IN1.low(); IN2.high()  # Left Motor Backward
    IN3.high(); IN4.low()  # Right Motor Forward


def start_turn_right_action(speed_percent_left, speed_percent_right):
    """Starts turning right (left motor forward/faster, right motor backward/slower)."""
    print(f"MOTORS: Start Turning right (actual) at L:{speed_percent_left}%, R:{speed_percent_right}%")
    set_speed_pwm(ENA, speed_percent_left)
    set_speed_pwm(ENB, speed_percent_right)
    IN1.high(); IN2.low()  # Left Motor Forward
    IN3.low(); IN4.high()  # Right Motor Backward

# Timed actions are generally not used by the new state machine, but kept for potential other uses.
def timed_turn_left_action(speed_percent_left, speed_percent_right, turn_duration_s):
    """Performs a timed left turn."""
    global robot_active_target_state
    print(f"MOTORS_TIMED: Turning left at L:{speed_percent_left}%, R:{speed_percent_right}% for {turn_duration_s}s.")
    start_turn_left_action(speed_percent_left, speed_percent_right)
    end_time = time.ticks_add(time.ticks_ms(), int(turn_duration_s * 1000))
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        if not robot_active_target_state: print("TIMED_TURN_LEFT: Stop signal."); break
        time.sleep_ms(10)
    stop_motors_action()


def timed_turn_right_action(speed_percent_left, speed_percent_right, turn_duration_s):
    """Performs a timed right turn."""
    global robot_active_target_state
    print(f"MOTORS_TIMED: Turning right at L:{speed_percent_left}%, R:{speed_percent_right}% for {turn_duration_s}s.")
    start_turn_right_action(speed_percent_left, speed_percent_right)
    end_time = time.ticks_add(time.ticks_ms(), int(turn_duration_s * 1000))
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        if not robot_active_target_state: print("TIMED_TURN_RIGHT: Stop signal."); break
        time.sleep_ms(10)
    stop_motors_action()


def timed_short_move_action(speed_percent_left, speed_percent_right, shift_duration_s):
    """Performs a timed short forward movement."""
    global robot_active_target_state
    print(f"MOTORS_TIMED: Short move at L:{speed_percent_left}%, R:{speed_percent_right}% for {shift_duration_s}s.")
    start_move_forward_action(speed_percent_left, speed_percent_right)
    end_time = time.ticks_add(time.ticks_ms(), int(shift_duration_s * 1000))
    while time.ticks_diff(end_time, time.ticks_ms()) > 0:
        if not robot_active_target_state: print("TIMED_SHORT_MOVE: Stop signal."); break
        time.sleep_ms(10)
    stop_motors_action()


def motor_buzz_action(start_buzz, speed_percent_left=MOTOR_BUZZ_SPEED_L, speed_percent_right=MOTOR_BUZZ_SPEED_R):
    """Activates or deactivates motor buzz effect (slow forward on both motors)."""
    global current_led_state
    if start_buzz:
        IN1.low(); IN2.low(); IN3.low(); IN4.low() # Stop other motors first
        set_speed_pwm(ENA, 0); set_speed_pwm(ENB, 0)
        time.sleep_ms(20)

        print(f"ROBOT_ACTION: Motor Buzz ON (L:{speed_percent_left}%, R:{speed_percent_right}%)")
        set_speed_pwm(ENA, speed_percent_left)
        set_speed_pwm(ENB, speed_percent_right)
        IN1.high(); IN2.low()
        IN3.high(); IN4.low()
        current_led_state = "RAPID_FLASH" # Visual feedback for buzz
    else:
        print("ROBOT_ACTION: Motor Buzz OFF")
        stop_motors_action()


def get_distance_action():
    """Measures distance using ultrasonic sensor."""
    TRIG.low(); time.sleep_us(2)
    TRIG.high(); time.sleep_us(10)
    TRIG.low()

    t_start_echo_low = time.ticks_us()
    while ECHO.value() == 0:
        if time.ticks_diff(time.ticks_us(), t_start_echo_low) > 30000: # 30ms timeout
            return 999
    t1_echo_high_start = time.ticks_us()

    t_start_echo_high = time.ticks_us()
    while ECHO.value() == 1:
        if time.ticks_diff(time.ticks_us(), t_start_echo_high) > 30000: # 30ms timeout
            return 999
    t2_echo_high_end = time.ticks_us()

    duration_us = time.ticks_diff(t2_echo_high_end, t1_echo_high_start)
    dist_cm = (duration_us * 0.0343) / 2
    return dist_cm if dist_cm >= 1 else 999


def perform_robot_start_sequence_actions():
    """Actions to perform when autonomous mode starts."""
    global current_led_state, current_autonomous_action, next_autonomous_logic_run_time_ms, manual_fan_is_on, action_end_time_ms
    print("ROBOT_SYSTEM: Autonomous Start Sequence Actions...")
    stop_motors_action() # Ensure motors are stopped initially
    FAN.high()
    manual_fan_is_on = True
    current_led_state = "SOLID_ON" # Solid LED for active autonomous mode
    update_led_indicator()

    current_autonomous_action = "INITIAL_PAUSE" # Start with the initial pause
    action_end_time_ms = time.ticks_add(time.ticks_ms(), int(AUTONOMOUS_INITIAL_PAUSE_S * 1000))
    next_autonomous_logic_run_time_ms = time.ticks_ms() # Allow process_autonomous_state to run immediately
    print(f"ROBOT_SYSTEM: Systems (Motors Stopped, Fan ON, LED Solid ON). Autonomous entering INITIAL_PAUSE for {AUTONOMOUS_INITIAL_PAUSE_S}s.")


def perform_robot_stop_sequence_actions():
    """Actions for a full system stop (manual or autonomous end)."""
    global current_led_state, manual_fan_is_on, current_autonomous_action
    print("ROBOT_SYSTEM: Perform Stop Sequence Actions (Full Stop)...")
    stop_motors_action()
    FAN.low()
    manual_fan_is_on = False
    current_led_state = "OFF" # LED off when fully stopped (unless BLE connected)
    # LED state will be re-evaluated by main loop based on ble_is_connected
    update_led_indicator()
    current_autonomous_action = "IDLE" # Reset autonomous state
    print("ROBOT_SYSTEM: All Systems (Motors, Fan) OFF. Autonomous state IDLE. LED state will update based on BLE.")


def update_led_indicator():
    """Updates the LED indicator based on current_led_state."""
    global last_led_toggle_time_ms
    current_time = time.ticks_ms()

    new_led_value = LED_WORK.value()

    if current_led_state == "OFF":
        new_led_value = 0
    elif current_led_state == "SOLID_ON":
        new_led_value = 1
    elif current_led_state == "SLOW_FLASH":
        if time.ticks_diff(current_time, last_led_toggle_time_ms) >= LED_SLOW_FLASH_INTERVAL_MS:
            new_led_value = 1 - LED_WORK.value()
            last_led_toggle_time_ms = current_time
    elif current_led_state == "RAPID_FLASH":
        if time.ticks_diff(current_time, last_led_toggle_time_ms) >= LED_RAPID_FLASH_INTERVAL_MS:
            new_led_value = 1 - LED_WORK.value()
            last_led_toggle_time_ms = current_time

    if LED_WORK.value() != new_led_value:
        LED_WORK.value(new_led_value)


# --- BLE UART Class ---
class BLEUART:
    def __init__(self, ble_obj, name=_ADVERTISING_NAME):
        self._ble = ble_obj
        self._ble.active(False); time.sleep_ms(100) # Reset BLE radio
        self._ble.active(True)
        print("BLE_UART: BLE Radio Activated.")
        self._ble.irq(self._irq_handler)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE_DEFINITION,))
        print(f"BLE_UART: Service registered. TX:{self._tx_handle}, RX:{self._rx_handle}")
        self._connections = set()
        name_bytes = name.encode()
        self._adv_payload = bytearray()
        self._adv_payload += b'\x02\x01\x06' # Flags: LE General Discoverable Mode, BR/EDR Not Supported
        self._adv_payload += bytes([len(name_bytes) + 1, 0x09]) + name_bytes # Complete Local Name
        self._advertise()

    def _advertise(self):
        print(f"BLE_UART: Advertising as '{_ADVERTISING_NAME}'...")
        try:
            self._ble.gap_advertise(100_000, adv_data=self._adv_payload) # Advertise indefinitely (interval 100ms)
        except Exception as e:
            print(f"BLE_UART_ERROR: Adv failed: {e}")

    def _irq_handler(self, event, data):
        global ble_is_connected, last_ble_command_str, last_ble_command_time_ms, current_led_state, last_led_toggle_time_ms
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            ble_is_connected = True
            print(f"BLE_UART_EVENT: Connected. Handle: {conn_handle}")
            self.send_message(f"Hi from {_ADVERTISING_NAME}!\n")
            # LED state will be updated in main loop based on ble_is_connected and robot_active_target_state
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections:
                self._connections.discard(conn_handle)
            if not self._connections: # All connections lost
                ble_is_connected = False
                print(f"BLE_UART_EVENT: Disconnected. All connections lost.")
                handle_ble_disconnect_safety() # Stop robot if it was in manual BLE mode
                self._advertise() # Re-advertise
            # LED state will be updated in main loop
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            if conn_handle in self._connections and value_handle == self._rx_handle:
                try:
                    received_bytes = self._ble.gatts_read(self._rx_handle)
                    last_ble_command_str = received_bytes.decode().strip().upper()
                    last_ble_command_time_ms = time.ticks_ms()
                    print(f"BLE_RX: '{last_ble_command_str}'")
                except Exception as e:
                    print(f"BLE_RX_ERROR: Decode failed: {e}"); last_ble_command_str = ""

    def send_message(self, data_str):
        if ble_is_connected:
            for conn_handle in self._connections:
                try:
                    self._ble.gatts_notify(conn_handle, self._tx_handle, data_str.encode())
                except Exception as e:
                    print(f"BLE_TX_ERROR: Notify for {conn_handle}: {e}")

    def is_ble_connected(self): # Public method to check connection status
        return ble_is_connected


# --- Safety Function for BLE Disconnect ---
def handle_ble_disconnect_safety():
    global robot_active_target_state, current_led_state, active_ble_movement_command
    print("SAFETY_HANDLER: BLE Disconnected.")

    # If robot was NOT in autonomous mode (i.e., potentially in manual BLE control)
    # and had an active movement command, stop it.
    if not robot_active_target_state and active_ble_movement_command:
        print("SAFETY_HANDLER: Was in manual BLE mode with active command. Stopping motors.")
        stop_motors_action() # This also clears active_ble_movement_command
        # Fan state is not changed here, only motor movements.

    # If autonomous was active, it continues. If it was off, it stays off.
    # perform_robot_stop_sequence_actions() is NOT called here UNLESS we want to force stop autonomous on BLE disconnect.
    # Current logic: autonomous continues, manual stops. LED will update in main loop.
    print("SAFETY_HANDLER: BLE disconnect processed. Autonomous state unaffected if active.")


# --- Autonomous Mode State Machine Logic ---
def process_autonomous_state():
    global robot_active_target_state, current_autonomous_action, action_end_time_ms, perform_left_turn_sequence_next

    # If autonomous mode is globally stopped, ensure motors are off and reset state.
    if not robot_active_target_state:
        # perform_robot_stop_sequence_actions() in main loop handles this transition more cleanly.
        # This is a safeguard within the autonomous processor.
        if current_autonomous_action != "IDLE":
            try: # Check if motors are running before printing/stopping again
                if (ENA.duty_u16() > 0 and (IN1.value() or IN2.value())) or \
                   (ENB.duty_u16() > 0 and (IN3.value() or IN4.value())):
                    print("AUTONOMOUS_PROCESSOR: Stop signal (robot_active_target_state is False), ensuring motors stopped.")
                    stop_motors_action()
            except Exception: pass # PWM might be deinit
            print(f"AUTONOMOUS_PROCESSOR: Robot not active. Current action '{current_autonomous_action}' reset to IDLE.")
            current_autonomous_action = "IDLE"
        return

    current_time = time.ticks_ms()

    # State: INITIAL_PAUSE (Pause after autonomous mode is activated)
    if current_autonomous_action == "INITIAL_PAUSE":
        # Motors are stopped by perform_robot_start_sequence_actions()
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: INITIAL_PAUSE complete -> MOVING_FORWARD.")
            current_autonomous_action = "MOVING_FORWARD"
            start_move_forward_action(AUTONOMOUS_FORWARD_SPEED_L, AUTONOMOUS_FORWARD_SPEED_R)
        # else: still pausing, do nothing

    # State: MOVING_FORWARD
    elif current_autonomous_action == "MOVING_FORWARD":
        measured_distance = get_distance_action()
        if not robot_active_target_state: return # Check again in case of stop during sensor read

        if measured_distance == 999: # Sensor error
            print(f"AUTONOMOUS_SENSOR_ERROR (MOVING_FORWARD). Pausing motors."); stop_motors_action()
            # Stays in MOVING_FORWARD, will retry sensor on next tick. Motors are stopped.
        else:
            print(f"AUTONOMOUS_STATE: MOVING_FORWARD. Dist: {measured_distance:.1f} cm")
            if measured_distance < AUTONOMOUS_OBSTACLE_THRESHOLD_CM:
                print(f"AUTONOMOUS_OBSTACLE @ {measured_distance:.1f} cm! -> OBSTACLE_PAUSING.")
                stop_motors_action() # Stop first
                current_autonomous_action = "OBSTACLE_PAUSING"
                action_end_time_ms = time.ticks_add(current_time, int(AUTONOMOUS_OBSTACLE_DETECT_PAUSE_S * 1000))
            else: # No obstacle, ensure still moving forward
                  # If motors were stopped due to a previous sensor error, restart them.
                try: # Check if motors are actually stopped before restarting
                    if ENA.duty_u16() == 0 and ENB.duty_u16() == 0 :
                         print("AUTONOMOUS_STATE: MOVING_FORWARD (restarting after sensor error pause).")
                         start_move_forward_action(AUTONOMOUS_FORWARD_SPEED_L, AUTONOMOUS_FORWARD_SPEED_R)
                    # If already moving, this call is redundant but harmless.
                except Exception: # PWM might be deinit (unlikely here)
                       start_move_forward_action(AUTONOMOUS_FORWARD_SPEED_L, AUTONOMOUS_FORWARD_SPEED_R)


    # State: OBSTACLE_PAUSING (Pause after detecting an obstacle)
    elif current_autonomous_action == "OBSTACLE_PAUSING":
        # Motors are stopped
        if not robot_active_target_state: return
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: OBSTACLE_PAUSING complete -> TURNING_STEP_1.")
            current_autonomous_action = "TURNING_STEP_1"
            if perform_left_turn_sequence_next:
                # Use new specific speeds for left turn
                start_turn_left_action(AUTONOMOUS_LEFT_TURN_MOTOR_L_SPEED, AUTONOMOUS_LEFT_TURN_MOTOR_R_SPEED)
            else:
                # Use new specific speeds for right turn
                start_turn_right_action(AUTONOMOUS_RIGHT_TURN_MOTOR_L_SPEED, AUTONOMOUS_RIGHT_TURN_MOTOR_R_SPEED)
            action_end_time_ms = time.ticks_add(current_time, int(TURN_DURATION_S * 1000))

    # State: TURNING_STEP_1 (First part of the turn maneuver)
    elif current_autonomous_action == "TURNING_STEP_1":
        if not robot_active_target_state: return
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: TURNING_STEP_1 complete -> POST_TURN_1_PAUSE.")
            stop_motors_action() # Stop before pausing
            current_autonomous_action = "POST_TURN_1_PAUSE"
            action_end_time_ms = time.ticks_add(current_time, int(AUTONOMOUS_POST_TURN_1_PAUSE_S * 1000))

    # State: POST_TURN_1_PAUSE (Pause after the first turn)
    elif current_autonomous_action == "POST_TURN_1_PAUSE":
        # Motors are stopped
        if not robot_active_target_state: return
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: POST_TURN_1_PAUSE complete -> SHIFTING_FORWARD.")
            current_autonomous_action = "SHIFTING_FORWARD"
            start_move_forward_action(AUTONOMOUS_SHORT_MOVE_SPEED_L, AUTONOMOUS_SHORT_MOVE_SPEED_R)
            action_end_time_ms = time.ticks_add(current_time, int(SHIFT_MOVE_DURATION_S * 1000))

    # State: SHIFTING_FORWARD (Short forward movement during maneuver)
    elif current_autonomous_action == "SHIFTING_FORWARD":
        if not robot_active_target_state: return
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: SHIFTING_FORWARD complete -> POST_SHIFT_PAUSE.")
            stop_motors_action() # Stop before pausing
            current_autonomous_action = "POST_SHIFT_PAUSE"
            action_end_time_ms = time.ticks_add(current_time, int(AUTONOMOUS_POST_SHIFT_PAUSE_S * 1000))

    # State: POST_SHIFT_PAUSE (Pause after the short forward shift)
    elif current_autonomous_action == "POST_SHIFT_PAUSE":
        # Motors are stopped
        if not robot_active_target_state: return
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: POST_SHIFT_PAUSE complete -> TURNING_STEP_2.")
            current_autonomous_action = "TURNING_STEP_2"
            # The turn direction logic here implies the *same effective turn direction* as STEP_1
            # or a continuation/adjustment. If perform_left_turn_sequence_next was true for STEP_1,
            # it's still true here for the second part of that same left-avoidance maneuver.
            if perform_left_turn_sequence_next:
                # Use new specific speeds for left turn
                start_turn_left_action(AUTONOMOUS_LEFT_TURN_MOTOR_L_SPEED, AUTONOMOUS_LEFT_TURN_MOTOR_R_SPEED)
            else:
                # Use new specific speeds for right turn
                start_turn_right_action(AUTONOMOUS_RIGHT_TURN_MOTOR_L_SPEED, AUTONOMOUS_RIGHT_TURN_MOTOR_R_SPEED)
            action_end_time_ms = time.ticks_add(current_time, int(TURN_DURATION_S * 1000))

    # State: TURNING_STEP_2 (Second part of the turn maneuver)
    elif current_autonomous_action == "TURNING_STEP_2":
        if not robot_active_target_state: return
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: TURNING_STEP_2 complete -> POST_TURN_2_PAUSE.")
            stop_motors_action() # Stop before pausing
            perform_left_turn_sequence_next = not perform_left_turn_sequence_next # Toggle for the *next* full obstacle maneuver
            current_autonomous_action = "POST_TURN_2_PAUSE"
            action_end_time_ms = time.ticks_add(current_time, int(AUTONOMOUS_POST_TURN_2_PAUSE_S * 1000))

    # State: POST_TURN_2_PAUSE (Pause after the second turn, before continuing)
    elif current_autonomous_action == "POST_TURN_2_PAUSE":
        # Motors are stopped
        if not robot_active_target_state: return
        if time.ticks_diff(action_end_time_ms, current_time) <= 0:
            print("AUTONOMOUS_STATE: POST_TURN_2_PAUSE complete. Maneuver finished -> MOVING_FORWARD (to continue).")
            current_autonomous_action = "MOVING_FORWARD" # Go back to moving forward
            start_move_forward_action(AUTONOMOUS_FORWARD_SPEED_L, AUTONOMOUS_FORWARD_SPEED_R)
            # No action_end_time_ms here as MOVING_FORWARD is continuous until obstacle or stop

    # State: IDLE (Fallback, should ideally not be reached if robot_active_target_state is True,
    # as perform_robot_start_sequence_actions sets INITIAL_PAUSE)
    elif current_autonomous_action == "IDLE" and robot_active_target_state:
        print("AUTONOMOUS_STATE: IDLE (but active). This should transition via button/BLE command. Forcing to INITIAL_PAUSE as a safeguard.")
        # This indicates a potential logic flaw if reached. Start sequence should handle it.
        # Forcing a restart of the autonomous sequence.
        perform_robot_start_sequence_actions() # This will set state to INITIAL_PAUSE and stop motors.

# --- Main Program Execution Loop ---
def main_program_loop():
    global robot_active_target_state, last_physical_button_action_time_ms
    global current_autonomous_action, next_autonomous_logic_run_time_ms
    global ble_uart_instance, ble_is_connected
    global last_ble_command_str, last_ble_command_time_ms, active_ble_movement_command
    global manual_fan_is_on, current_led_state, last_led_toggle_time_ms

    print("\n--- ROBOT CONTROL (Autonomous & BLE Integrated) ---")
    print(f"Button Pin: GP{BUTTON_PIN_NUMBER}. BLE Name: '{_ADVERTISING_NAME}'")
    print(f"AUTONOMOUS PARAMS: FwdSpd L/R={AUTONOMOUS_FORWARD_SPEED_L}%/{AUTONOMOUS_FORWARD_SPEED_R}%")
    # Print new turn speeds
    print(f"  LeftTurnSpd (MotorL/MotorR)={AUTONOMOUS_LEFT_TURN_MOTOR_L_SPEED}%/{AUTONOMOUS_LEFT_TURN_MOTOR_R_SPEED}%")
    print(f"  RightTurnSpd (MotorL/MotorR)={AUTONOMOUS_RIGHT_TURN_MOTOR_L_SPEED}%/{AUTONOMOUS_RIGHT_TURN_MOTOR_R_SPEED}%")
    print(f"  ShortMoveSpd L/R={AUTONOMOUS_SHORT_MOVE_SPEED_L}%/{AUTONOMOUS_SHORT_MOVE_SPEED_R}%, ObstacleThreshold={AUTONOMOUS_OBSTACLE_THRESHOLD_CM}cm")
    print(f"  Delays (s): Initial={AUTONOMOUS_INITIAL_PAUSE_S}, ObstacleDetect={AUTONOMOUS_OBSTACLE_DETECT_PAUSE_S}, PostTurn1={AUTONOMOUS_POST_TURN_1_PAUSE_S}, PostShift={AUTONOMOUS_POST_SHIFT_PAUSE_S}, PostTurn2={AUTONOMOUS_POST_TURN_2_PAUSE_S}")
    print(f"MANUAL PARAMS: FwdSpd L/R={MANUAL_FORWARD_SPEED_L}%/{MANUAL_FORWARD_SPEED_R}%, TurnSpd L/R={MANUAL_TURN_SPEED_L}%/{MANUAL_TURN_SPEED_R}%, BuzzSpd L/R={MOTOR_BUZZ_SPEED_L}%/{MOTOR_BUZZ_SPEED_R}%")

    try:
        ble_radio_obj = bluetooth.BLE()
        ble_uart_instance = BLEUART(ble_radio_obj)
        print("BLE UART Initialized and Advertising.")
    except Exception as e:
        print(f"CRITICAL_ERROR: Failed to initialize BLE: {e}. Operating in physical button mode only.")
        ble_uart_instance = None
        ble_is_connected = False # Ensure this is false if BLE init fails

    # Initial full stop and state setup
    robot_active_target_state = False 
    perform_robot_stop_sequence_actions() 

    while True:
        current_time_ms = time.ticks_ms()

        # 1. Handle Physical Button Press for Autonomous Toggle
        if BTN_START_STOP.value() == 0: # Button pressed (active low)
            if time.ticks_diff(current_time_ms, last_physical_button_action_time_ms) > BUTTON_DEBOUNCE_PERIOD_MS:
                last_physical_button_action_time_ms = current_time_ms # Debounce start
                print("PHYSICAL_BUTTON: Press detected. Waiting for release...")
                time.sleep_ms(20) # Short delay for stable read
                while BTN_START_STOP.value() == 0: time.sleep_ms(10) # Wait for release
                print("PHYSICAL_BUTTON: Released.")
                last_physical_button_action_time_ms = time.ticks_ms() # Debounce end after release
                
                robot_active_target_state = not robot_active_target_state
                if robot_active_target_state:
                    print("PHYSICAL_BUTTON: Toggled Autonomous Mode ON.")
                    perform_robot_start_sequence_actions()
                else:
                    print("PHYSICAL_BUTTON: Toggled Autonomous Mode OFF.")
                    perform_robot_stop_sequence_actions()
        
        # 2. Handle Incoming BLE Commands
        command_from_ble = ""
        if ble_is_connected and last_ble_command_str:
            command_from_ble = last_ble_command_str
            last_ble_command_str = "" # Consume command

        if ble_is_connected:
            if command_from_ble == "CLEAN": # Toggle Autonomous via BLE
                robot_active_target_state = not robot_active_target_state
                if robot_active_target_state:
                    print("BLE_COMMAND: 'CLEAN' toggled Autonomous Mode ON.")
                    perform_robot_start_sequence_actions()
                else:
                    print("BLE_COMMAND: 'CLEAN' toggled Autonomous Mode OFF.")
                    perform_robot_stop_sequence_actions()
            
            elif not robot_active_target_state: # Manual BLE Control Mode (only if autonomous is OFF)
                if command_from_ble: # A new command was received for manual control
                    # If the new command is to toggle OFF the current action
                    if command_from_ble == active_ble_movement_command or \
                       (command_from_ble == "HORN" and active_ble_movement_command == "HORN") or \
                       (command_from_ble == "FORWARD" and active_ble_movement_command == "FORWARD_OBSTACLE"):
                        
                        print(f"BLE_COMMAND_TOGGLE: Turning OFF '{active_ble_movement_command}' (received '{command_from_ble}')")
                        if active_ble_movement_command == "HORN" or active_ble_movement_command == "FORWARD_OBSTACLE":
                            motor_buzz_action(False) # This calls stop_motors_action()
                        else:
                            stop_motors_action()
                        # active_ble_movement_command is now None (set by stop_motors_action or motor_buzz_action(False))

                    # Else, the new command is to toggle ON a new action (or switch actions)
                    else:
                        # Stop any currently active motor action first, unless it's FAN_TOGGLE
                        if command_from_ble != "FAN_TOGGLE" and active_ble_movement_command is not None:
                            print(f"BLE_COMMAND_SWITCH: Stopping previous action '{active_ble_movement_command}' before starting '{command_from_ble}'")
                            if active_ble_movement_command == "HORN" or active_ble_movement_command == "FORWARD_OBSTACLE":
                                motor_buzz_action(False)
                            else: # Any other motor movement
                                stop_motors_action()
                            # active_ble_movement_command is now None

                        # Now, process the new command_from_ble to turn it ON
                        print(f"BLE_COMMAND_TOGGLE: Turning ON '{command_from_ble}'")
                        if command_from_ble == "FORWARD":
                            dist = get_distance_action()
                            if dist < MANUAL_OBSTACLE_THRESHOLD_CM and dist != 999: # Check sensor isn't erroring
                                print(f"BLE_MANUAL_OBSTACLE: Forward blocked by obstacle at {dist:.1f}cm.")
                                motor_buzz_action(True, MOTOR_BUZZ_SPEED_L, MOTOR_BUZZ_SPEED_R) 
                                active_ble_movement_command = "FORWARD_OBSTACLE"
                            else:
                                start_move_forward_action(MANUAL_FORWARD_SPEED_L, MANUAL_FORWARD_SPEED_R)
                                active_ble_movement_command = "FORWARD"
                        elif command_from_ble == "BACKWARDS":
                            start_move_backward_action(MANUAL_FORWARD_SPEED_L, MANUAL_FORWARD_SPEED_R)
                            active_ble_movement_command = "BACKWARDS"
                        elif command_from_ble == "LEFT": # User wants "LEFT" button to turn robot right
                            print("BLE_COMMAND: LEFT received, executing RIGHT turn.")
                            start_turn_right_action(MANUAL_TURN_SPEED_L, MANUAL_TURN_SPEED_R)
                            active_ble_movement_command = "LEFT"
                        elif command_from_ble == "RIGHT": # User wants "RIGHT" button to turn robot left
                            print("BLE_COMMAND: RIGHT received, executing LEFT turn.")
                            start_turn_left_action(MANUAL_TURN_SPEED_L, MANUAL_TURN_SPEED_R)
                            active_ble_movement_command = "RIGHT"
                        elif command_from_ble == "HORN":
                            motor_buzz_action(True, MOTOR_BUZZ_SPEED_L, MOTOR_BUZZ_SPEED_R)
                            active_ble_movement_command = "HORN"
                        elif command_from_ble == "FAN_TOGGLE":
                            manual_fan_is_on = not manual_fan_is_on
                            FAN.value(manual_fan_is_on)
                            print(f"BLE_COMMAND: Fan Toggled {'ON' if manual_fan_is_on else 'OFF'}")
                            # FAN_TOGGLE does not change active_ble_movement_command or stop other movements
                        else:
                            print(f"BLE_COMMAND: Received unknown/unhandled command '{command_from_ble}'. No change to active movement.")
        
        # 3. Execute Autonomous Logic Step (if active and interval passed)
        if robot_active_target_state:
            if time.ticks_diff(current_time_ms, next_autonomous_logic_run_time_ms) >= 0:
                process_autonomous_state()
                next_autonomous_logic_run_time_ms = time.ticks_add(current_time_ms, AUTONOMOUS_LOGIC_INTERVAL_MS)
        
        # 4. Set LED State based on overall robot state
        previous_led_state = current_led_state # Store to check if it changed
        
        if robot_active_target_state: # Autonomous mode is ON
            current_led_state = "SOLID_ON" # Solid ON for autonomous
        elif ble_is_connected: # Manual BLE mode or Idle with BLE connection
            if active_ble_movement_command == "HORN" or active_ble_movement_command == "FORWARD_OBSTACLE":
                current_led_state = "RAPID_FLASH" # Buzzing/Obstacle alert
            else: # Includes None (idle but connected) or any other non-buzzing movement
                current_led_state = "SLOW_FLASH" # Indicates BLE connected and ready/moving manually
        else: # Idle, no BLE connection, not autonomous
            current_led_state = "OFF"
        
        # If LED state changed, reset toggle timer for immediate visual feedback of new flash pattern
        if current_led_state != previous_led_state:
            last_led_toggle_time_ms = current_time_ms 
            # Ensure flash starts with LED ON if transitioning to a flash state and LED was off
            if (current_led_state == "SLOW_FLASH" or current_led_state == "RAPID_FLASH") and LED_WORK.value() == 0 :
                 LED_WORK.high() 
            elif current_led_state == "SOLID_ON":
                 LED_WORK.high()
            elif current_led_state == "OFF":
                 LED_WORK.low()

        update_led_indicator() # Handles the actual toggling for flashing states

        # 5. Failsafe checks (optional, but good practice)
        if not robot_active_target_state and active_ble_movement_command is None: # Supposed to be completely idle
            try:
                if (ENA.duty_u16() > 0 and (IN1.value() or IN2.value())) or \
                   (ENB.duty_u16() > 0 and (IN3.value() or IN4.value())):
                    print("MAIN_LOOP_FAILSAFE: Motors appear active when robot should be idle. Forcing stop.")
                    perform_robot_stop_sequence_actions() # This will stop motors and set fan correctly
            except Exception: # PWM might be deinitialized after cleanup, ignore error then
                pass 
            
            # Check fan state only if not in manual fan mode and robot is idle
            # perform_robot_stop_sequence_actions already handles fan if called above.
            # This is an additional check if the above motor check didn't trigger a full stop.
            if not manual_fan_is_on and FAN.value() == 1: # Fan is ON but manual_fan_is_on is False (and not autonomous)
                 print("MAIN_LOOP_FAILSAFE: Fan active when it should be off (idle, manual fan off). Forcing fan off.")
                 FAN.low()


        time.sleep_ms(MAIN_LOOP_POLL_DELAY_MS)

# --- Program Entry Point ---
if __name__ == "__main__":
    try:
        main_program_loop()
    except KeyboardInterrupt:
        print("\nPROGRAM_FLOW: KeyboardInterrupt (Ctrl+C).")
    except Exception as e:
        print(f"\nPROGRAM_FLOW_CRITICAL_ERROR: An unhandled exception: {e}")
        import sys
        sys.print_exception(e)
    finally:
        print("\nPROGRAM_FLOW: Entering final cleanup...")
        robot_active_target_state = False # Ensure this is set for stop sequence
        perform_robot_stop_sequence_actions() # Stop motors, fan, set LED off, set autonomous state to IDLE
        
        if ble_uart_instance and hasattr(ble_uart_instance, '_ble'):
            try:
                if ble_uart_instance._ble.active(): # Check if active before trying to deactivate
                    print("BLE_CLEANUP: Deactivating BLE radio.")
                    ble_uart_instance._ble.active(False)
            except Exception as e:
                print(f"BLE_CLEANUP_ERROR: {e}")
        
        print("ROBOT_CLEANUP: De-initializing PWMs...")
        try:
            if 'ENA' in globals() and isinstance(ENA, PWM): ENA.deinit()
        except Exception as e: print(f"ENA deinit error: {e}")
        try:
            if 'ENB' in globals() and isinstance(ENB, PWM): ENB.deinit()
        except Exception as e: print(f"ENB deinit error: {e}")
        print("ROBOT_CLEANUP: PWMs de-initialized (or attempted).")
        print("--- SYSTEM HALTED ---")