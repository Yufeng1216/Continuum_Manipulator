"""
Raspberry Pi pressure controller for the pneumatic chambers.
Reads a target pressure (Pa) from UART (/dev/ttyS0, 9600 baud) and drives the
inflation/deflation pumps and solenoid valves with a PID loop (plus leak compensation
and feedforward), using a BMP280 sensor for feedback.

Requires a BMP280 helper module (`atmosphere_function1`, not included) that provides
BMP280(...) and read_pressure().
"""
import RPi.GPIO as GPIO
import time
import serial
from atmosphere_function1 import *

# GPIO Pins for the first motor
ENB1 = 18  # Enable signal for motor 1
INC1 = 23  # Signal input 1 for motor 1
IND1 = 24  # Signal input 2 for motor 1

# GPIO Pins for the second motor
ENB2 = 25  # Enable signal for motor 2
INC2 = 16  # Signal input 1 for motor 2
IND2 = 20  # Signal input 2 for motor 2

# GPIO Pins for the first chamber solenoid valves
Relay1_1 = 5  # Relay 1 for inflation of chamber 1
Relay1_2 = 6  # Relay 2 for deflation of chamber 1

# GPIO Pins for the second chamber solenoid valves
Relay2_1 = 13  # Relay 1 for inflation of chamber 2
Relay2_2 = 19  # Relay 2 for deflation of chamber 2

# PID Controller parameters
KP = 0.35
KI = 0.005
KD = 0
leak_rate = 0.01
feedforward = 1000

GPIO.setwarnings(False)  # Disable warning messages

# Initialize UART
def init_uart():
    uart = serial.Serial('/dev/ttyS0', baudrate=9600, timeout=1)  # 初始化 UART 串口
    return uart

# Function to read set_p value from UART
def read_set_p_from_uart(uart):
    try:
        if uart.in_waiting > 0:  # Check if data is available
            set_p = uart.readline().decode('utf-8').strip()  # Read and decode the set_p value
            return int(set_p)
    except Exception as e:
        print(f"Error reading from UART: {e}")
    return None

# Initialize GPIO
def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(ENB1, GPIO.OUT)
    GPIO.setup(INC1, GPIO.OUT)
    GPIO.setup(IND1, GPIO.OUT)
    GPIO.setup(ENB2, GPIO.OUT)
    GPIO.setup(INC2, GPIO.OUT)
    GPIO.setup(IND2, GPIO.OUT)
    GPIO.setup(Relay1_1, GPIO.OUT)
    GPIO.setup(Relay1_2, GPIO.OUT)
    GPIO.setup(Relay2_1, GPIO.OUT)
    GPIO.setup(Relay2_2, GPIO.OUT)

# Initialize PWM (Pulse Width Modulation)
def pwm(enable_pin):
    pwm_instance = GPIO.PWM(enable_pin, 500)
    pwm_instance.start(0)
    return pwm_instance

# Change motor speed by changing duty cycle
def changespeed(pwm_instance, speed_percentage):
    duty_cycle = int(speed_percentage * 100)  # Convert the speed percentage to a duty cycle
    pwm_instance.ChangeDutyCycle(duty_cycle)  # Set the duty cycle of the PWM instance

# Motor clockwise rotation signal
def clockwise(in1_pin, in2_pin):
    GPIO.output(in1_pin, 1)
    GPIO.output(in2_pin, 0)

# Motor counter-clockwise rotation signal
def counter_clockwise(in1_pin, in2_pin):
    GPIO.output(in1_pin, 0)
    GPIO.output(in2_pin, 1)

# Motor brake signal
def brake(pwm_instance, in1_pin, in2_pin):
    GPIO.output(in1_pin, 0)
    GPIO.output(in2_pin, 0)
    changespeed(pwm_instance, 0)

# Control solenoid valve state for each chamber
def control_solenoid_valve(chamber, state):
    if chamber == 1:
        inflation_relay = Relay1_1
        deflation_relay = Relay1_2
    elif chamber == 2:
        inflation_relay = Relay2_1
        deflation_relay = Relay2_2
    else:
        print("Invalid chamber number")
        return

    if state == 'inflate':
        GPIO.output(inflation_relay, GPIO.HIGH)
        GPIO.output(deflation_relay, GPIO.LOW)
        print("inflate, relay1_high")
    elif state == 'deflate':
        GPIO.output(inflation_relay, GPIO.LOW)
        GPIO.output(deflation_relay, GPIO.HIGH)
    elif state == 'off':
        GPIO.output(inflation_relay, GPIO.LOW)
        GPIO.output(deflation_relay, GPIO.LOW)
    else:
        print("Invalid state for solenoid valve control")

# PID Controller
class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.set_point = 0
        self.int_error = 0
        self.prev_error = 0

    def update(self, measured_value):
        error = self.set_point - measured_value
        self.int_error += error
        derivative = error - self.prev_error
        self.prev_error = error
        return self.kp * error + self.ki * self.int_error + self.kd * derivative

# Set pressure in a chamber using PID control with leak compensation and feedforward control
def set_pressure(chamber, expect_p, leak_rate=0, feedforward=0):
    pid = PID(KP, KI, KD)  # Initialize the PID controller with global gains
    pid.set_point = expect_p  # Set the desired pressure as the set point

    while True:
        current_p = int(bmp280.read_pressure())  # Read the current pressure, assuming bmp280 is initialized
        error = int(pid.update(current_p))  # Calculate the error between the set point and the current pressure

        # Leak rate compensation: Add a fixed leak rate to the error (assuming linear relation)
        adjusted_error = error + leak_rate  # Compensate for leak rate

        # Frontfeed control: Use a proportional feedforward control to anticipate leak adjustments
        adjusted_error += feedforward  # Compensate for known pressure loss rate due to leak

        # Map the error to a speed percentage, assuming the maximum possible error is max_error
        max_error = 4000  # Assume the maximum pressure error is 4000 Pa
        if adjusted_error > 0:
            speed_percentage = adjusted_error / max_error if abs(adjusted_error) < max_error else 1
        elif adjusted_error < 0:
            speed_percentage = (-adjusted_error) / (max_error) if abs(adjusted_error) < max_error else 1
        else:
            speed_percentage = 0

        # Ensure the speed_percentage is within the range of 0 to 1
        speed_percentage = max(0, min(speed_percentage, 1))

        # Control solenoid valve state (inflate or deflate based on error)
        control_solenoid_valve(chamber, 'inflate' if adjusted_error > 0 else 'deflate')

        # Control the pump speed based on the adjusted error
        if expect_p > 101325:
            changespeed(ENB_pwm1, speed_percentage)
        elif expect_p <= 101325:
            counter_clockwise(INC2, IND2)
            changespeed(ENB_pwm2, speed_percentage)
            print("haha\n")

        print(f"error:{adjusted_error}\n")
        print(f"speed: {speed_percentage:.2f}\n")

        time.sleep(0.1)  # Adjust the sleep time based on your needs

# Main function to control motors and solenoid valves
def control_motors_and_valves():
    setup()  # Initialize GPIO pins
    ENB_pwm1 = pwm(ENB1)  # Initialize PWM for motor 1
    ENB_pwm2 = pwm(ENB2)  # Initialize PWM for motor 2
    bmp280 = BMP280(BMP280_ULTRAHIGHRES)  # Initialize BMP280 sensor
    uart = init_uart()  # Initialize UART
    
    while True:
        # Read the set_p value from UART
        set_p = read_set_p_from_uart(uart)
        if set_p is not None:
            print(f"Received set_p value: {set_p}")
            set_pressure(1, set_p, leak_rate, feedforward)  # Set pressure in chamber 1 with leak rate compensation and feedforward control

        time.sleep(1)  # Adjust the sleep time based on your needs

# Clean up the GPIO at the end of the program
if __name__ == '__main__':
    control_motors_and_valves()
