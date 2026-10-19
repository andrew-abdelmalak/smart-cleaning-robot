"""Smart Cleaning Robot firmware for the Raspberry Pi Pico W (MicroPython).

Drives a two-wheel differential robot with an L298N motor driver, single-channel
optical wheel encoders, an HC-SR04 ultrasonic sensor, a suction fan, a status
LED, and a push button. Commands arrive over a Nordic UART BLE service. Motion
uses feed-forward PWM plus PID correction on the encoder counts.
"""

import time

import bluetooth
import ubinascii
from machine import PWM, Pin, unique_id
from micropython import const

# ===========================================================
# BLUETOOTH CONSTANTS
# ===========================================================

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_UART_SERVICE_UUID = bluetooth.UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E")
_UART_TX_CHAR_UUID = (
    bluetooth.UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E"),
    bluetooth.FLAG_NOTIFY,
)
_UART_RX_CHAR_UUID = (
    bluetooth.UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E"),
    bluetooth.FLAG_WRITE,
)
_UART_SERVICE_DEFINITION = (
    _UART_SERVICE_UUID,
    (_UART_TX_CHAR_UUID, _UART_RX_CHAR_UUID),
)

# Unique device name from the last two bytes of the board ID, e.g. PicoRobot-3C33
_ADVERTISING_NAME = "PicoRobot-" + ubinascii.hexlify(unique_id()[-2:]).decode().upper()

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
STALL_REVERSE_MS = 700

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

# Encoder counts at the start of a straight run or a pivot
straight_start_left, straight_start_right = 0, 0
pivot_start_left, pivot_start_right = 0, 0

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

# Last time either motor had non-zero duty, from time.ticks_ms()
last_motor_time = 0

# ===========================================================
# ENCODER HANDLERS
# ===========================================================


def left_encoder_isr(pin: Pin) -> None:
    """Count one rising edge of the left wheel encoder."""
    global left_pulses
    left_pulses += 1


def right_encoder_isr(pin: Pin) -> None:
    """Count one rising edge of the right wheel encoder."""
    global right_pulses
    right_pulses += 1


left_enc.irq(trigger=Pin.IRQ_RISING, handler=left_encoder_isr)
right_enc.irq(trigger=Pin.IRQ_RISING, handler=right_encoder_isr)

# ===========================================================
# MOTOR CONTROL
# ===========================================================


def ff_pwm(pps_target: float, wheel: str) -> int:
    """Estimate the PWM duty that drives one wheel at a target speed.

    A linear motor model, duty = kS + kV * speed, fitted per wheel. kS covers
    static friction and kV the speed gain.

    Args:
        pps_target: Target wheel speed in encoder pulses per second.
        wheel: "L" for the left wheel; any other value means the right wheel.

    Returns:
        Duty cycle in percent, capped at 95.
    """
    if wheel == "L":
        kS, kV = 27, 0.66
    else:
        kS, kV = 30, 0.61
    return min(95, int(kS + kV * pps_target))


def set_motor_pwm(left_pwm: float, right_pwm: float, direction: str = "stop") -> None:
    """Set both motor duties and the H-bridge direction pins.

    Args:
        left_pwm: Left wheel duty in percent (0 to 100).
        right_pwm: Right wheel duty in percent (0 to 100).
        direction: "forward", "backward", "pivot_left", or "pivot_right".
            Any other value stops both motors.
    """
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


def _pid(gains: dict, error: float, integral: float, last_error: float) -> tuple:
    """Run one PID update with the integral clamped to INTEGRAL_CLAMP_LIMIT.

    Args:
        gains: Dict with "P", "I", and "D" gains.
        error: Current error.
        integral: Integral before this update.
        last_error: Error from the previous update.

    Returns:
        (adjustment, new_integral).
    """
    integral += error
    if abs(integral) > INTEGRAL_CLAMP_LIMIT:
        integral = INTEGRAL_CLAMP_LIMIT if integral > 0 else -INTEGRAL_CLAMP_LIMIT
    adjustment = (
        gains["P"] * error + gains["I"] * integral + gains["D"] * (error - last_error)
    )
    return adjustment, integral


def _drive_pps(left_pps: float, right_pps: float, direction: str) -> None:
    """Drive both wheels at target speeds through the feed-forward model."""
    set_motor_pwm(ff_pwm(abs(left_pps), "L"), ff_pwm(abs(right_pps), "R"), direction)


def straight_motion(
    direction: str, start_left: int | None = None, start_right: int | None = None
) -> None:
    """Drive straight, with PID correction when start counts are given.

    The PID error is the difference in pulses each wheel has travelled since
    the start counts. Without start counts the drive is feed-forward only.

    Args:
        direction: "forward" or "backward".
        start_left: Left encoder count at the start of the run.
        start_right: Right encoder count at the start of the run.
    """
    global straight_integral, straight_last_error

    if start_left is None or start_right is None:
        _drive_pps(MOVEMENT_PPS, MOVEMENT_PPS * RIGHT_TRIM, direction)
        return

    # Positive when the left wheel has travelled further
    error = (left_pulses - start_left) - (right_pulses - start_right)
    adjustment, straight_integral = _pid(
        GAINS_STRAIGHT, error, straight_integral, straight_last_error
    )
    straight_last_error = error

    _drive_pps(
        MOVEMENT_PPS - adjustment, MOVEMENT_PPS * RIGHT_TRIM + adjustment, direction
    )


def pivot_motion(side: str, angle_pulses: int | None = None) -> None:
    """Pivot in place, with two PID loops when a target angle is given.

    The progress PID drives the total pulse count toward angle_pulses. The
    balance PID adjusts the split between the wheels. Without a target the
    pivot is feed-forward only.

    Args:
        side: "left" or "right".
        angle_pulses: Target pivot size in total encoder pulses.
    """
    global progress_integral, progress_last_error
    global balance_integral, balance_last_error

    direction = "pivot_left" if side == "left" else "pivot_right"

    if angle_pulses is None:
        _drive_pps(MOVEMENT_PPS, MOVEMENT_PPS * RIGHT_TRIM, direction)
        return

    left_delta = left_pulses - pivot_start_left
    right_delta = right_pulses - pivot_start_right

    # Progress PID: pulses still to go
    progress_error = angle_pulses - (abs(left_delta) + abs(right_delta))
    progress_adj, progress_integral = _pid(
        GAINS_TURN_PROGRESS, progress_error, progress_integral, progress_last_error
    )
    progress_last_error = progress_error

    # Balance PID. Both encoders count up only, so this sums the deltas
    # rather than differencing them (see README, Known limitations).
    if side == "left":
        balance_error = -left_delta - right_delta
    else:
        balance_error = left_delta + right_delta

    balance_adj, balance_integral = _pid(
        GAINS_TURN_BALANCE, balance_error, balance_integral, balance_last_error
    )
    balance_last_error = balance_error

    base_power = MOVEMENT_PPS + progress_adj
    _drive_pps(
        base_power + balance_adj, base_power * RIGHT_TRIM - balance_adj, direction
    )


def horn_buzz() -> None:
    """Buzz the motors at low duty as an audible horn."""
    buzz_duty = 10
    set_motor_pwm(buzz_duty, buzz_duty * RIGHT_TRIM, "forward")


def apply_brake() -> None:
    """Brake with a short reverse pulse, then stop both motors."""
    set_motor_pwm(BRAKE_DUTY_PERCENT, BRAKE_DUTY_PERCENT * RIGHT_TRIM, "backward")
    time.sleep_ms(BRAKE_DURATION_MS)
    set_motor_pwm(0, 0, "stop")


# ===========================================================
# MOVEMENT STATE CONTROL
# ===========================================================


def stop_all_movement() -> None:
    """Stop the motors and clear the manual command and autonomous state."""
    global manual_command, autonomous_state
    manual_command = None
    autonomous_state = "IDLE"
    set_motor_pwm(0, 0, "stop")


def motors_idle() -> bool:
    """Return True when both motors have zero duty."""
    return ena.duty_u16() == 0 and enb.duty_u16() == 0


def is_full_stop() -> bool:
    """Return True when both motors are off and have been for 100 ms."""
    return motors_idle() and time.ticks_diff(time.ticks_ms(), last_motor_time) >= 100


def emergency_stop() -> None:
    """Stop everything, leave autonomous mode, and reset all PID state."""
    global autonomous_mode, horn_active, forward_blocked

    stop_all_movement()
    autonomous_mode = horn_active = forward_blocked = False
    start_straight()
    start_pivot()
    fan.off()


# ===========================================================
# SENSORS & FEEDBACK
# ===========================================================


def get_distance_cm() -> float:
    """Measure the distance to the nearest obstacle with the HC-SR04.

    Returns:
        Distance in cm, or 999 when no echo starts or ends within 30 ms.
        A disconnected sensor therefore reads as a clear path (see README,
        Known limitations).
    """
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


def toggle_horn() -> None:
    """Toggle the horn.

    The horn buzzes the motors, so the robot never reads as fully stopped
    while it sounds. Switching it on needs a full stop; switching it off
    is always allowed.
    """
    global horn_active

    if not horn_active and not is_full_stop():
        return

    horn_active = not horn_active
    if not horn_active:
        stop_all_movement()


def update_led() -> None:
    """Set the status LED pattern from the robot state.

    Rapid flash: obstacle block or horn. Solid: autonomous mode. Slow flash:
    BLE connected. Off: idle.
    """
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
    else:
        period = LED_SLOW_FLASH_MS if led_state == "SLOW_FLASH" else LED_RAPID_FLASH_MS
        if time.ticks_diff(current, last_led_toggle) >= period:
            led.toggle()
            last_led_toggle = current


# ===========================================================
# BLUETOOTH UART CLASS
# ===========================================================


class BLEUART:
    """Nordic UART Service peripheral that receives text commands.

    Each write to the RX characteristic is decoded, stripped, upper-cased,
    and stored in last_ble_command for the control loop. Losing the last
    connection triggers an emergency stop.
    """

    def __init__(self, ble_obj: bluetooth.BLE, name: str = _ADVERTISING_NAME) -> None:
        self._ble = ble_obj
        self._ble.active(True)
        self._ble.irq(self._irq_handler)

        # Register service
        ((_, self._rx_handle),) = self._ble.gatts_register_services(
            (_UART_SERVICE_DEFINITION,)
        )
        self._connections = set()

        # Prepare advertising
        self._adv_payload = (
            bytearray(b"\x02\x01\x06") + bytes([len(name) + 1, 0x09]) + name.encode()
        )
        self._advertise()

    def _advertise(self) -> None:
        """Start connectable advertising every 100 ms."""
        try:
            self._ble.gap_advertise(100_000, adv_data=self._adv_payload)
        except OSError as e:
            print("[BLE] advertise failed:", e)

    def _irq_handler(self, event: int, data: tuple) -> None:
        """Track connections and store each received command."""
        global ble_connected, last_ble_command

        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            ble_connected = True

        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.discard(conn_handle)
            if not self._connections:
                ble_connected = False
                last_ble_command = ""  # never run a command after reconnect
                emergency_stop()
            # Advertising stops on connect; restart it so a phone can reconnect.
            self._advertise()

        elif event == _IRQ_GATTS_WRITE:
            try:
                last_ble_command = (
                    self._ble.gatts_read(self._rx_handle).decode().strip().upper()
                )
            except UnicodeError:
                last_ble_command = ""


# ===========================================================
# MAIN PROGRAM LOOP
# ===========================================================


def read_command() -> str:
    """Return the next command from BLE or the button.

    A button press (active low, 50 ms debounce) maps to CLEAN and blocks
    until release, so one press gives one command.

    Returns:
        The command string, or "" when there is none.
    """
    global last_ble_command

    command = ""
    if ble_connected and last_ble_command:
        command = last_ble_command
        last_ble_command = ""

    if button.value() == 0:
        time.sleep_ms(50)
        if button.value() == 0:
            command = "CLEAN"
            while button.value() == 0:
                time.sleep_ms(10)
    return command


def handle_command(command: str) -> bool:
    """Apply one user command to the robot state.

    Args:
        command: One of CLEAN, FAN, HORN, FORWARD, BACKWARD, LEFT, RIGHT.
            Unknown commands are ignored.

    Returns:
        True when the rest of this loop iteration must be skipped.
    """
    global autonomous_mode, autonomous_state, manual_command
    global forward_blocked, horn_active

    if command == "CLEAN":
        if not is_full_stop():
            emergency_stop()
            return True
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
        toggle_horn()

    elif command == "FORWARD" and forward_blocked:
        # Clear the block if the path is now free, else silence the horn.
        if get_distance_cm() >= OBSTACLE_THRESHOLD_CM:
            forward_blocked = False
            horn_active = False
            return True
        if horn_active:
            horn_active = False
            return True

    elif forward_blocked and command in ("BACKWARD", "LEFT", "RIGHT"):
        # Moving away from the obstacle clears the block.
        forward_blocked = False
        horn_active = False

    # Manual movement: repeating the active command brakes and stops.
    if not autonomous_mode and command in ("FORWARD", "BACKWARD", "LEFT", "RIGHT"):
        if manual_command == command:
            apply_brake()
            manual_command = None
        else:
            stop_all_movement()
            manual_command = command
            if command == "FORWARD":
                start_straight()
    return False


def drive_manual() -> bool:
    """Run one step of the active manual command.

    FORWARD uses encoder PID and stops with the horn when an obstacle is
    closer than OBSTACLE_THRESHOLD_CM. The other commands are feed-forward.

    Returns:
        True when an obstacle blocked FORWARD and the iteration must end.
    """
    global forward_blocked, horn_active, manual_command

    if manual_command == "FORWARD":
        if get_distance_cm() < OBSTACLE_THRESHOLD_CM:
            forward_blocked = True
            horn_active = True
            manual_command = None
            return True
        straight_motion("forward", straight_start_left, straight_start_right)
    elif manual_command == "BACKWARD":
        straight_motion("backward")
    elif manual_command == "LEFT":
        pivot_motion("left")
    elif manual_command == "RIGHT":
        pivot_motion("right")
    return False


def start_straight() -> None:
    """Record the straight-run start counts and reset the straight PID."""
    global straight_start_left, straight_start_right
    global straight_integral, straight_last_error

    straight_start_left = left_pulses
    straight_start_right = right_pulses
    straight_integral = straight_last_error = 0


def start_pivot() -> None:
    """Record the pivot start counts and reset both pivot PID controllers."""
    global pivot_start_left, pivot_start_right
    global progress_integral, progress_last_error
    global balance_integral, balance_last_error

    pivot_start_left = left_pulses
    pivot_start_right = right_pulses
    progress_integral = progress_last_error = 0
    balance_integral = balance_last_error = 0


def run_autonomous_step(now: int) -> bool:
    """Advance the autonomous cleaning state machine by one step.

    The robot drives straight (FWD) until an obstacle is closer than
    OBSTACLE_THRESHOLD_CM, then pivots left TURN_PULSES encoder pulses
    (AVOID) and drives on. If a wheel stops counting for STALL_TIME_MS, it
    reverses, brakes, and makes the same pivot (STALL_TURN).

    Args:
        now: Timestamp of this loop iteration from time.ticks_ms().

    Returns:
        True when the iteration must end early, after a brake.
    """
    global autonomous_state
    global stall_start_time, last_check_left, last_check_right
    global next_stall_check

    if autonomous_state == "START_FWD":
        start_straight()
        stall_start_time = 0
        last_check_left = left_pulses
        last_check_right = right_pulses
        next_stall_check = time.ticks_add(now, STALL_CHECK_INTERVAL_MS)
        autonomous_state = "FWD"

    elif autonomous_state == "FWD":
        if get_distance_cm() < OBSTACLE_THRESHOLD_CM:
            apply_brake()
            autonomous_state = "START_AVOID"
            return True

        straight_motion("forward", straight_start_left, straight_start_right)

        # Stall check: a wheel whose count has not changed since the last
        # check is stalled; recover once the stall lasts STALL_TIME_MS.
        if time.ticks_diff(now, next_stall_check) >= 0:
            if left_pulses == last_check_left or right_pulses == last_check_right:
                if stall_start_time == 0:
                    stall_start_time = now
                elif time.ticks_diff(now, stall_start_time) > STALL_TIME_MS:
                    print("STALL DETECTED! RECOVERING...")
                    straight_motion("backward")
                    time.sleep_ms(STALL_REVERSE_MS)
                    apply_brake()
                    start_pivot()
                    autonomous_state = "STALL_TURN"
                    return True
            else:
                stall_start_time = 0

            last_check_left = left_pulses
            last_check_right = right_pulses
            next_stall_check = time.ticks_add(now, STALL_CHECK_INTERVAL_MS)

    elif autonomous_state == "START_AVOID":
        start_pivot()
        autonomous_state = "AVOID"

    elif autonomous_state == "AVOID" or autonomous_state == "STALL_TURN":
        turned = (left_pulses - pivot_start_left) + (right_pulses - pivot_start_right)
        if abs(turned) >= TURN_PULSES:
            apply_brake()
            autonomous_state = "START_FWD"
        else:
            pivot_motion("left", TURN_PULSES)
    return False


def control_step() -> None:
    """Run one 20 ms iteration of the control loop.

    Priority order: the horn, then the active manual command,
    then the autonomous state machine, then a safety stop when idle.
    """
    now = time.ticks_ms()

    command = read_command()
    if command and handle_command(command):
        return

    if horn_active:
        horn_buzz()
        update_led()
        time.sleep_ms(20)
        return

    if manual_command:
        if drive_manual():
            return
    elif autonomous_mode:
        if run_autonomous_step(now):
            return
    elif not motors_idle():
        stop_all_movement()

    update_led()
    time.sleep_ms(20)


def main_program_loop() -> None:
    """Register the BLE UART service, then run the control loop forever."""
    _ble_uart = BLEUART(bluetooth.BLE())  # keep a reference to the service
    while True:
        control_step()


# ===========================================================
# ENTRY POINT
# ===========================================================

if __name__ == "__main__":
    try:
        main_program_loop()
    except Exception as e:
        print(f"[FATAL] Exception: {e}")
    finally:
        print("[SYSTEM] Program terminated.")
        emergency_stop()
        led.off()
