import json
import socket
import threading
import time
import sys

# --- GPIO Imports and Motor Driver Setup ---
# ONLY RUN THIS ON RASPBERRY PI
try:
    import RPi.GPIO as GPIO

    # Use BCM pin numbering
    GPIO.setmode(GPIO.BCM)

    # Define GPIO pins for L298N Motor Driver (from your reference code)
    MOTOR_A_IN1 = 19 # Left Motor IN1
    MOTOR_A_IN2 = 20 # Left Motor IN2
    MOTOR_A_ENA = 26 # Left Motor ENA (PWM pin for speed control)

    MOTOR_B_IN3 = 6  # Right Motor IN3
    MOTOR_B_IN4 = 5  # Right Motor IN4
    MOTOR_B_ENB = 13 # Right Motor ENB (PWM pin for speed control)

    # Pen Control (placeholder - add your pen mechanism here)
    # PEN_CONTROL_PIN = 21 # Example GPIO for a solenoid/servo
    # GPIO.setup(PEN_CONTROL_PIN, GPIO.OUT)
    # GPIO.output(PEN_CONTROL_PIN, GPIO.LOW) # Pen initially UP

    GPIO.setup([MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_ENA,
                MOTOR_B_IN3, MOTOR_B_IN4, MOTOR_B_ENB], GPIO.OUT)

    PWM_FREQUENCY = 100 # Hz for motor PWM
    pwm_a = GPIO.PWM(MOTOR_A_ENA, PWM_FREQUENCY)
    pwm_b = GPIO.PWM(MOTOR_B_ENB, PWM_FREQUENCY)

    pwm_a.start(0)
    pwm_b.start(0)

    MOTOR_ENABLED = True
    print("RPi.GPIO and motors initialized.")
except ImportError:
    print("RPi.GPIO not found. Running in simulation mode (motor commands will be printed).")
    MOTOR_ENABLED = False
except Exception as e:
    print(f"RPi Error: Setting up RPi.GPIO: {e}. Running in simulation mode.")
    MOTOR_ENABLED = False

# --- Network Configuration ---
RPi_LISTEN_PORT = 5006 # Must match ROBOT_UDP_PORT_SEND_COMMANDS in PC code

# --- Global Variables ---
running = True
command_queue = []
command_lock = threading.Lock()

# --- Motor Control Functions ---
def set_motor_speed(pwm_obj, speed):
    """Sets PWM duty cycle, clamps speed between 0-100."""
    if MOTOR_ENABLED:
        pwm_obj.ChangeDutyCycle(max(0, min(100, int(speed))))

def stop_motors():
    """Immediately stops both motors."""
    print("RPi: Stopping motors.")
    if MOTOR_ENABLED:
        GPIO.output([MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_B_IN3, MOTOR_B_IN4], GPIO.LOW)
        set_motor_speed(pwm_a, 0)
        set_motor_speed(pwm_b, 0)
    else:
        print("SIMULATED: Stop motors")

def execute_movement_command(command, speed):
    """
    Executes movement commands sent from PC.
    Commands: "forward", "left", "right", "fast_left", "fast_right", "stop"
    """
    print(f"RPi: Executing {command} at speed {speed}")
    
    if command == "stop":
        stop_motors()
        
    elif command == "forward":
        # Both motors forward at same speed
        if MOTOR_ENABLED:
            # Left Motor Forward
            GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_A_IN2, GPIO.LOW)
            set_motor_speed(pwm_a, speed)
            
            # Right Motor Forward
            GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
            GPIO.output(MOTOR_B_IN4, GPIO.LOW)
            set_motor_speed(pwm_b, speed)
        else:
            print(f"SIMULATED: Forward at speed {speed}")
            
    elif command == "left":
        # Normal left turn: left motor slower, right motor normal speed
        left_speed = int(speed * 0.3)  # Slow down left motor
        right_speed = speed
        if MOTOR_ENABLED:
            # Left Motor Slower Forward
            GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_A_IN2, GPIO.LOW)
            set_motor_speed(pwm_a, left_speed)
            
            # Right Motor Normal Forward
            GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
            GPIO.output(MOTOR_B_IN4, GPIO.LOW)
            set_motor_speed(pwm_b, right_speed)
        else:
            print(f"SIMULATED: Left turn - Left: {left_speed}, Right: {right_speed}")
            
    elif command == "right":
        # Normal right turn: right motor slower, left motor normal speed
        left_speed = speed
        right_speed = int(speed * 0.3)  # Slow down right motor
        if MOTOR_ENABLED:
            # Left Motor Normal Forward
            GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_A_IN2, GPIO.LOW)
            set_motor_speed(pwm_a, left_speed)
            
            # Right Motor Slower Forward
            GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
            GPIO.output(MOTOR_B_IN4, GPIO.LOW)
            set_motor_speed(pwm_b, right_speed)
        else:
            print(f"SIMULATED: Right turn - Left: {left_speed}, Right: {right_speed}")
            
    elif command == "fast_left":
        # Fast left turn: left motor backward, right motor forward
        if MOTOR_ENABLED:
            # Left Motor Backward
            GPIO.output(MOTOR_A_IN1, GPIO.LOW)
            GPIO.output(MOTOR_A_IN2, GPIO.HIGH)
            set_motor_speed(pwm_a, speed)
            
            # Right Motor Forward
            GPIO.output(MOTOR_B_IN3, GPIO.HIGH)
            GPIO.output(MOTOR_B_IN4, GPIO.LOW)
            set_motor_speed(pwm_b, speed)
        else:
            print(f"SIMULATED: Fast left turn - Left: -{speed}, Right: {speed}")
            
    elif command == "fast_right":
        # Fast right turn: left motor forward, right motor backward
        if MOTOR_ENABLED:
            # Left Motor Forward
            GPIO.output(MOTOR_A_IN1, GPIO.HIGH)
            GPIO.output(MOTOR_A_IN2, GPIO.LOW)
            set_motor_speed(pwm_a, speed)
            
            # Right Motor Backward
            GPIO.output(MOTOR_B_IN3, GPIO.LOW)
            GPIO.output(MOTOR_B_IN4, GPIO.HIGH)
            set_motor_speed(pwm_b, speed)
        else:
            print(f"SIMULATED: Fast right turn - Left: {speed}, Right: -{speed}")
    else:
        print(f"RPi: Unknown command: {command}")

def control_pen(state):
    """
    Controls pen up/down mechanism.
    state: 0 for pen UP, 1 for pen DOWN.
    Add your pen control hardware here (servo, solenoid, etc.)
    """
    if state == 1:
        # if MOTOR_ENABLED and 'PEN_CONTROL_PIN' in globals():
        #     GPIO.output(PEN_CONTROL_PIN, GPIO.HIGH) # Activate for pen down
        print("RPi: Pen DOWN")
    else:
        # if MOTOR_ENABLED and 'PEN_CONTROL_PIN' in globals():
        #     GPIO.output(PEN_CONTROL_PIN, GPIO.LOW) # Deactivate for pen up
        print("RPi: Pen UP")

# --- UDP Communication Functions ---
def receive_commands_udp_thread():
    """Thread function to continuously receive UDP commands from the PC."""
    global command_queue, running
    print(f"RPi: Listening for UDP commands on port {RPi_LISTEN_PORT}")
    
    rpi_listen_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        rpi_listen_socket.bind(("", RPi_LISTEN_PORT))
        rpi_listen_socket.settimeout(0.1)
    except Exception as e:
        print(f"RPi Error: Binding UDP listen socket: {e}")
        running = False
        return

    while running:
        try:
            data, addr = rpi_listen_socket.recvfrom(1024)
            received_message = data.decode('utf-8').strip()
            
            with command_lock:
                command_queue.append(received_message)
            
        except socket.timeout:
            pass
        except Exception as e:
            print(f"RPi Error: receiving UDP command: {e}")
            break
    
    rpi_listen_socket.close()
    print("RPi: UDP command reception thread terminated.")

# --- Main Application Loop ---
def main():
    global running

    # Start UDP command reception thread
    command_thread = threading.Thread(target=receive_commands_udp_thread, daemon=True)
    command_thread.start()

    print("\n--- Doodlebot RPi Motor Control - PC Command Format ---")
    print("Waiting for motor commands from PC...")
    print("Expected format: {'command': 'forward/left/right/fast_left/fast_right/stop', 'speed': 50, 'pen_state': 1}")
    print("Press Ctrl+C to exit.")

    try:
        while running:
            # Process Commands from PC
            commands_to_process = []
            with command_lock:
                if command_queue:
                    commands_to_process = list(command_queue)
                    command_queue.clear()

            for cmd_str in commands_to_process:
                try:
                    command_json = json.loads(cmd_str)
                    
                    # Extract command parameters (matching PC format)
                    if "command" in command_json and "speed" in command_json and "pen_state" in command_json:
                        command = command_json["command"]
                        speed = command_json["speed"]
                        pen_state = command_json["pen_state"]
                        
                        print(f"RPi: Received - Command: {command}, Speed: {speed}, Pen: {pen_state}")
                        
                        # Execute movement command
                        execute_movement_command(command, speed)
                        
                        # Control pen
                        control_pen(pen_state)
                        
                    else:
                        print(f"RPi: Invalid command format: {command_json}")
                        print("RPi: Expected: {'command': 'forward', 'speed': 50, 'pen_state': 1}")

                except json.JSONDecodeError as e:
                    print(f"RPi: JSON decode error: {e} for command: {cmd_str}")
                except Exception as e:
                    print(f"RPi Error: Processing PC command '{cmd_str}': {e}")
            
            time.sleep(0.01)  # Small delay to reduce CPU usage

    except KeyboardInterrupt:
        print("\nRPi: Interrupted by user.")
    except Exception as e:
        print(f"RPi: An unexpected error occurred in main loop: {e}")
    finally:
        print("RPi: Cleaning up...")
        stop_motors()
        if MOTOR_ENABLED:
            pwm_a.stop()
            pwm_b.stop()
            GPIO.cleanup()
        print("RPi: Application terminated.")
        running = False
        sys.exit(0)

if name == "main":
    main()
