import time
from machine import Pin, PWM, unique_id
import bluetooth
from micropython import const
import ubinascii

# =====================================================================================
# BLUETOOTH CONFIGURATION
# =====================================================================================

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

# =====================================================================================
# HARDWARE PIN DEFINITIONS
# =====================================================================================

# Motor Control Pins
ENA_PIN, IN1_PIN, IN2_PIN = 6, 12, 13
ENB_PIN, IN3_PIN, IN4_PIN = 7, 14, 15

# Encoder Input Pins
PHYSICAL_LEFT_ENCODER_PIN = 21
PHYSICAL_RIGHT_ENCODER_PIN = 20

# Peripheral Device Pins
FAN_PIN = 8
LED_WORK_PIN = 18
TRIG_PIN = 10
ECHO_PIN = 11
BUTTON_PIN_NUMBER = 16

# =====================================================================================
# SYSTEM CONFIGURATION PARAMETERS
# =====================================================================================

# Motion Control Parameters
SPEED_PPS = 100
OBSTACLE_THRESHOLD_CM = 15
MOTOR_BUZZ_POWER = 15

# Navigation Constants
PULSES_FOR_120_DEG_TURN = 50
PULSES_FOR_REVERSE = 50

# Stall Detection Thresholds
STALL_POWER_THRESHOLD = 95.0
STALL_SPEED_THRESHOLD = 10
STALL_TIME_MS = 1000

# PID Control Parameters
PHYSICAL_LEFT_PID_GAINS = {"P": 0.075, "I": 0.014, "D": 4.0}
PHYSICAL_RIGHT_PID_GAINS = {"P": 0.065, "I": 0.016, "D": 3.5}
INTEGRAL_CLAMP = 15.0
POWER_CHANGE_LIMIT = 3.5
DEADBAND_TOLERANCE = 2.0
SAMPLE_TIME_S = 0.08

# LED Flash Timing
LED_SLOW_FLASH_INTERVAL_MS = 500
LED_RAPID_FLASH_INTERVAL_MS = 100

# =====================================================================================
# HARDWARE INITIALIZATION
# =====================================================================================

# Initialize Motor PWM Controllers
ena = PWM(Pin(ENA_PIN))
enb = PWM(Pin(ENB_PIN))
ena.freq(1000)
enb.freq(1000)

# Initialize Motor Direction Control Pins
in1 = Pin(IN1_PIN, Pin.OUT)
in2 = Pin(IN2_PIN, Pin.OUT)
in3 = Pin(IN3_PIN, Pin.OUT)
in4 = Pin(IN4_PIN, Pin.OUT)

# Initialize Encoder Input Pins
left_enc_pin = Pin(PHYSICAL_LEFT_ENCODER_PIN, Pin.IN, Pin.PULL_UP)
right_enc_pin = Pin(PHYSICAL_RIGHT_ENCODER_PIN, Pin.IN, Pin.PULL_UP)

# Initialize Peripheral Device Pins
fan = Pin(FAN_PIN, Pin.OUT)
led = Pin(LED_WORK_PIN, Pin.OUT)
trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN)
button = Pin(BUTTON_PIN_NUMBER, Pin.IN, Pin.PULL_UP)

# =====================================================================================
# GLOBAL STATE VARIABLES
# =====================================================================================

# Encoder Pulse Counters
left_wheel_pulses = 0
right_wheel_pulses = 0

# Robot Operation State
active_movement_command = None
robot_in_autonomous_mode = False
autonomous_state = "IDLE"

# Communication State
ble_is_connected = False
last_ble_command_str = ""

# User Interface State
current_led_state = "OFF"
last_led_toggle_time_ms = 0
horn_active_until = 0
is_forward_blocked = False

# System Monitoring
stall_check_start_time = 0

# =====================================================================================
# ENCODER INTERRUPT HANDLERS
# =====================================================================================

def left_encoder_callback(pin):
    global left_wheel_pulses
    left_wheel_pulses += 1

def right_encoder_callback(pin):
    global right_wheel_pulses
    right_wheel_pulses += 1

left_enc_pin.irq(trigger=Pin.IRQ_RISING, handler=left_encoder_callback)
right_enc_pin.irq(trigger=Pin.IRQ_RISING, handler=right_encoder_callback)

# =====================================================================================
# PID CONTROLLER CLASS
# =====================================================================================

class VelocityPID:
    """PID controller for maintaining target wheel velocity"""
    
    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.power = 0.0
        self.integral = 0.0
        self.last_error = 0.0
        self.last_pulse_count = 0
        self.last_power = 0.0
        self.current_pps = 0.0
    
    def reset(self):
        """Reset PID controller to initial state"""
        print("[PID] Controller Reset")
        self.power, self.integral, self.last_error = 0.0, 0.0, 0.0
        self.last_pulse_count, self.last_power = 0, 0.0
    
    def update(self, current_total_pulses, target_pps):
        """Update PID controller with current pulse count and target speed"""
        # Calculate current wheel velocity
        pulses_since_last_sample = current_total_pulses - self.last_pulse_count
        self.last_pulse_count = current_total_pulses
        self.current_pps = pulses_since_last_sample / SAMPLE_TIME_S
        
        # Calculate PID error with deadband
        error = target_pps - self.current_pps
        if abs(error) < DEADBAND_TOLERANCE:
            error *= 0.5
        
        # Update integral term with clamping
        self.integral += error * SAMPLE_TIME_S
        self.integral = max(-INTEGRAL_CLAMP, min(INTEGRAL_CLAMP, self.integral))
        
        # Calculate derivative term
        derivative = (error - self.last_error) / SAMPLE_TIME_S
        self.last_error = error
        
        # Calculate power adjustment with rate limiting
        adjustment = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        new_power = self.power + adjustment
        
        power_change = new_power - self.last_power
        if abs(power_change) > POWER_CHANGE_LIMIT:
            power_change = POWER_CHANGE_LIMIT if power_change > 0 else -POWER_CHANGE_LIMIT
        
        self.power = self.last_power + power_change
        self.power = max(0, min(100, self.power))
        self.last_power = self.power

        print(f"[PID] TGT:{target_pps:.1f} | ACT:{self.current_pps:.1f} | ERR:{error:.1f} | PWR:{self.power:.1f}")
        
        return self.power

# =====================================================================================
# MOTOR CONTROL FUNCTIONS
# =====================================================================================

def set_physical_motor_speeds(p_left_pwr, p_right_pwr, direction):
    """Control motor speeds and direction with physical wiring compensation"""
    # Compensate for physical motor wiring (swap left/right)
    s_left_pwr = p_right_pwr
    s_right_pwr = p_left_pwr
    
    print(f"[MOTOR] Dir: {direction}, L_Pwr: {p_left_pwr:.1f}, R_Pwr: {p_right_pwr:.1f}")

    # Set motor direction pins
    if direction == "forward":
        in3.high(); in4.low()
        in1.high(); in2.low()
    elif direction == "backward":
        in3.low(); in4.high()
        in1.low(); in2.high()
    elif direction == "pivot_left":
        in3.low(); in4.high()
        in1.high(); in2.low()
    elif direction == "pivot_right":
        in3.high(); in4.low()
        in1.low(); in2.high()
    else:  # stop
        in1.low(); in2.low()
        in3.low(); in4.low()
    
    # Apply PWM duty cycles
    l_duty = int(max(0, min(100, abs(s_left_pwr))) / 100 * 65535)
    r_duty = int(max(0, min(100, abs(s_right_pwr))) / 100 * 65535)
    ena.duty_u16(l_duty)
    enb.duty_u16(r_duty)

def stop_all_movement():
    """Stop all robot movement and reset control systems"""
    global active_movement_command, autonomous_state, is_forward_blocked
    print("[SYSTEM] Stop All Movement")
    active_movement_command = None
    autonomous_state = "IDLE"
    is_forward_blocked = False
    left_pid.reset()
    right_pid.reset()
    set_physical_motor_speeds(0, 0, "stop")

# =====================================================================================
# SENSOR FUNCTIONS
# =====================================================================================

def get_distance_cm():
    """Measure distance using ultrasonic sensor"""
    trig.low()
    time.sleep_us(2)
    trig.high()
    time.sleep_us(10)
    trig.low()
    
    t_start = time.ticks_us()
    while echo.value() == 0:
        if time.ticks_diff(time.ticks_us(), t_start) > 30000: 
            return 999
    
    t1 = time.ticks_us()
    while echo.value() == 1:
        if time.ticks_diff(time.ticks_us(), t1) > 30000: 
            return 999
    
    t2 = time.ticks_us()
    distance = (time.ticks_diff(t2, t1) * 0.0343) / 2
    print(f"[SENSOR] Distance: {distance:.1f} cm")
    return distance

# =====================================================================================
# USER INTERFACE FUNCTIONS
# =====================================================================================

def trigger_horn_oneshot():
    """Activate horn for one second and stop all movement"""
    global horn_active_until
    print("[SYSTEM] Horn Triggered")
    stop_all_movement()
    horn_active_until = time.ticks_add(time.ticks_ms(), 1000)

def update_led():
    """Update LED state based on robot status"""
    global last_led_toggle_time_ms, current_led_state
    
    current_time = time.ticks_ms()
    is_horn_timer_active = time.ticks_diff(horn_active_until, current_time) > 0
    
    # Determine appropriate LED state
    new_state = "OFF"
    if is_forward_blocked or is_horn_timer_active:
        new_state = "RAPID_FLASH"
    elif robot_in_autonomous_mode:
        new_state = "SOLID_ON"
    elif ble_is_connected:
        new_state = "SLOW_FLASH"
    
    if new_state != current_led_state:
        last_led_toggle_time_ms = current_time
        current_led_state = new_state
    
    # Execute LED behavior pattern
    if current_led_state == "OFF":
        led.off()
    elif current_led_state == "SOLID_ON":
        led.on()
    elif current_led_state == "SLOW_FLASH":
        if time.ticks_diff(current_time, last_led_toggle_time_ms) >= LED_SLOW_FLASH_INTERVAL_MS:
            led.toggle()
            last_led_toggle_time_ms = current_time
    elif current_led_state == "RAPID_FLASH":
        if time.ticks_diff(current_time, last_led_toggle_time_ms) >= LED_RAPID_FLASH_INTERVAL_MS:
            led.toggle()
            last_led_toggle_time_ms = current_time

# =====================================================================================
# BLUETOOTH COMMUNICATION CLASS
# =====================================================================================

class BLEUART:
    """Bluetooth UART service for robot remote control"""
    
    def __init__(self, ble_obj, name=_ADVERTISING_NAME):
        self._ble = ble_obj
        self._ble.active(True)
        self._ble.irq(self._irq_handler)
        ((self._tx_handle, self._rx_handle),) = self._ble.gatts_register_services((_UART_SERVICE_DEFINITION,))
        self._connections = set()
        self._adv_payload = bytearray(b'\x02\x01\x06') + bytes([len(name) + 1, 0x09]) + name.encode()
        self._advertise()
    
    def _advertise(self):
        """Start Bluetooth advertising"""
        try: 
            self._ble.gap_advertise(100_000, adv_data=self._adv_payload)
        except Exception as e: 
            print(f"[BLE] Advertise Error: {e}")
    
    def _irq_handler(self, event, data):
        """Handle Bluetooth events"""
        global ble_is_connected, last_ble_command_str
        
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            ble_is_connected = True
            print("[BLE] Central Connected")
            
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            if conn_handle in self._connections: 
                self._connections.discard(conn_handle)
            if not self._connections:
                ble_is_connected = False
                print("[BLE] Central Disconnected")
                stop_all_movement()
                
        elif event == _IRQ_GATTS_WRITE:
            conn_handle, value_handle = data
            try:
                received_bytes = self._ble.gatts_read(self._rx_handle)
                last_ble_command_str = received_bytes.decode().strip().upper()
                print(f"[BLE] RX Command: '{last_ble_command_str}'")
            except Exception as e:
                last_ble_command_str = ""
                print(f"[BLE] RX Error: {e}")

# =====================================================================================
# AUTONOMOUS NAVIGATION FUNCTIONS
# =====================================================================================

def execute_obstacle_avoidance():
    """Execute 120-degree left turn to avoid obstacles"""
    print("[AUTO] Executing obstacle avoidance turn...")
    start_L, start_R = left_wheel_pulses, right_wheel_pulses
    while (left_wheel_pulses - start_L + right_wheel_pulses - start_R) / 2 < PULSES_FOR_120_DEG_TURN:
        set_physical_motor_speeds(40, 40, "pivot_left")
        time.sleep_ms(10)
    stop_all_movement()

def execute_stall_recovery():
    """Execute reverse and turn sequence for stall recovery"""
    print("[AUTO] Executing stall recovery - reversing...")
    start_L, start_R = left_wheel_pulses, right_wheel_pulses
    while (left_wheel_pulses - start_L + right_wheel_pulses - start_R) / 2 < PULSES_FOR_REVERSE:
        set_physical_motor_speeds(40, 40, "backward")
        time.sleep_ms(10)
    
    print("[AUTO] Stall recovery - turning...")
    start_L, start_R = left_wheel_pulses, right_wheel_pulses
    while (left_wheel_pulses - start_L + right_wheel_pulses - start_R) / 2 < PULSES_FOR_120_DEG_TURN:
        set_physical_motor_speeds(40, 40, "pivot_left")
        time.sleep_ms(10)
    stop_all_movement()

# =====================================================================================
# MAIN PROGRAM EXECUTION
# =====================================================================================

def main_program_loop():
    """Main control loop for robot operation"""
    global active_movement_command, last_ble_command_str, ble_is_connected
    global robot_in_autonomous_mode, autonomous_state, stall_check_start_time
    global is_forward_blocked, left_pid, right_pid
    
    print("[SYSTEM] Initializing Smart Cleaning Robot...")
    
    # Initialize PID controllers for left and right wheels
    left_pid = VelocityPID(PHYSICAL_LEFT_PID_GAINS["P"], PHYSICAL_LEFT_PID_GAINS["I"], PHYSICAL_LEFT_PID_GAINS["D"])
    right_pid = VelocityPID(PHYSICAL_RIGHT_PID_GAINS["P"], PHYSICAL_RIGHT_PID_GAINS["I"], PHYSICAL_RIGHT_PID_GAINS["D"])
    
    # Initialize Bluetooth communication
    ble_uart_instance = BLEUART(bluetooth.BLE())
    last_pid_sample_time = time.ticks_ms()

    while True:
        # =========================================================================
        # INPUT PROCESSING SECTION
        # =========================================================================
        command = ""
        
        # Process Bluetooth commands
        if ble_is_connected and last_ble_command_str:
            command = last_ble_command_str
            last_ble_command_str = ""
        
        # Process physical button input
        if button.value() == 0:
            time.sleep_ms(50)  # Debounce delay
            if button.value() == 0:
                command = "CLEAN"
                print("[INPUT] Physical Button -> CLEAN")
                while button.value() == 0: 
                    time.sleep_ms(10)  # Wait for button release
        
        # =========================================================================
        # COMMAND EXECUTION SECTION
        # =========================================================================
        if command:
            # Clear obstacle alert for non-forward movements
            if is_forward_blocked and command != "FORWARD":
                is_forward_blocked = False
                stop_all_movement()

            if command == "CLEAN":
                stop_all_movement()
                robot_in_autonomous_mode = not robot_in_autonomous_mode
                if robot_in_autonomous_mode:
                    print("[MODE] Autonomous Cleaning Activated")
                    fan.on()
                    autonomous_state = "FORWARD"
                else:
                    print("[MODE] Manual Control Activated")
                    fan.off()
                continue

            elif command == "FAN":
                if fan.value():
                    fan.off()
                    print("[SYSTEM] Fan OFF")
                else:
                    fan.on()
                    print("[SYSTEM] Fan ON")

            elif not robot_in_autonomous_mode:
                if command in ["FORWARD", "BACKWARD", "LEFT", "RIGHT"]:
                    if active_movement_command == command:
                        stop_all_movement()
                    else:
                        stop_all_movement()
                        active_movement_command = command
                elif command == "HORN":
                    trigger_horn_oneshot()

        # =========================================================================
        # SYSTEM EXECUTION SECTION
        # =========================================================================
        current_time = time.ticks_ms()
        is_horn_timer_active = time.ticks_diff(horn_active_until, current_time) > 0
        
        # Handle alert states (horn or obstacle)
        if is_horn_timer_active or is_forward_blocked:
            set_physical_motor_speeds(MOTOR_BUZZ_POWER, MOTOR_BUZZ_POWER, "forward")
            
        # Handle autonomous cleaning mode
        elif robot_in_autonomous_mode:
            if autonomous_state == "FORWARD":
                # Check for obstacles
                if get_distance_cm() < OBSTACLE_THRESHOLD_CM:
                    stop_all_movement()
                    autonomous_state = "AVOIDING"
                    print("[AUTO] Obstacle detected - avoiding")
                    continue
                
                # Execute PID-controlled forward movement
                if time.ticks_diff(current_time, last_pid_sample_time) >= SAMPLE_TIME_S * 1000:
                    left_power = left_pid.update(left_wheel_pulses, SPEED_PPS)
                    right_power = right_pid.update(right_wheel_pulses, SPEED_PPS)
                    last_pid_sample_time = current_time
                    set_physical_motor_speeds(left_pid.power, right_pid.power, "forward")

                    # Monitor for stall conditions
                    is_stalled = left_pid.power > STALL_POWER_THRESHOLD and left_pid.current_pps < STALL_SPEED_THRESHOLD
                    if is_stalled:
                        if stall_check_start_time == 0:
                            stall_check_start_time = current_time
                            print("[STALL] Potential stall detected")
                        if time.ticks_diff(current_time, stall_check_start_time) > STALL_TIME_MS:
                            print("[STALL] Stall confirmed - initiating recovery")
                            stop_all_movement()
                            autonomous_state = "RECOVERING"
                            continue
                    else:
                        if stall_check_start_time != 0: 
                            print("[STALL] Condition cleared")
                        stall_check_start_time = 0
            
            elif autonomous_state == "AVOIDING":
                execute_obstacle_avoidance()
                autonomous_state = "FORWARD"
                print("[AUTO] Obstacle avoidance complete")
            
            elif autonomous_state == "RECOVERING":
                execute_stall_recovery()
                autonomous_state = "FORWARD"
                print("[AUTO] Stall recovery complete")

        # Handle manual control mode
        elif active_movement_command:
            direction, target_pps = "stop", 0
            
            if active_movement_command == "FORWARD":
                if get_distance_cm() < OBSTACLE_THRESHOLD_CM:
                    print("[MANUAL] Forward blocked by obstacle")
                    is_forward_blocked = True
                    active_movement_command = None
                    continue
                direction, target_pps = "forward", SPEED_PPS
                
            elif active_movement_command == "BACKWARD":
                direction, target_pps = "backward", SPEED_PPS
            elif active_movement_command == "LEFT":
                direction, target_pps = "pivot_left", SPEED_PPS
            elif active_movement_command == "RIGHT":
                direction, target_pps = "pivot_right", SPEED_PPS

            if time.ticks_diff(current_time, last_pid_sample_time) >= SAMPLE_TIME_S * 1000:
                left_pid.update(left_wheel_pulses, target_pps)
                right_pid.update(right_wheel_pulses, target_pps)
                last_pid_sample_time = current_time
                set_physical_motor_speeds(left_pid.power, right_pid.power, direction)
        
        # Handle idle state
        else:
            stop_all_movement()
            
        # Update user interface
        update_led()
        time.sleep_ms(10)

# =====================================================================================
# PROGRAM ENTRY POINT
# =====================================================================================

if __name__ == "__main__":
    try:
        main_program_loop()
    except Exception as e:
        print(f"[FATAL] Exception occurred: {e}")
    finally:
        print("[SYSTEM] Program terminated - stopping all systems")
        stop_all_movement()
        fan.low()
        led.low()