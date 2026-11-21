#!/usr/bin/env python3
"""
Keyboard teleop for the 4 Feetech STS servos only (C2 tendon winders).
"""
import os
import time

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

sys.path.append("..")
from scservo_sdk import *  # Uses SCServo SDK library

# Default settings
BAUDRATE = 1000000           # SCServo default baudrate : 1000000
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
                             # ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"

SCS_MINIMUM_POSITION_VALUE = 0  # SCServo will rotate between this value
SCS_MAXIMUM_POSITION_VALUE = 28672
SCS_MOVING_SPEED = 600         # SCServo moving speed
SCS_MOVING_ACC = 50           # SCServo moving acceleration
step = 200

# Initialize servo positions
servo1_pos = 2800
servo2_pos = 704
servo3_pos = 2200
servo4_pos = 2600

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

# Open port
if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    print("Press any key to terminate...")
    getch()
    quit()

# Set port baudrate
if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    print("Press any key to terminate...")
    getch()
    quit()

# Function to write positions to the servos
def sync_write_positions():
    # Send the updated positions to all servos
    print(f"Servo1: {servo1_pos}, Servo2: {servo2_pos}, Servo3: {servo3_pos}, Servo4: {servo4_pos}")
    packetHandler.SyncWritePosEx(1, servo1_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.SyncWritePosEx(2, servo2_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.SyncWritePosEx(3, servo3_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.SyncWritePosEx(4, servo4_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.groupSyncWrite.txPacket()
    packetHandler.groupSyncWrite.clearParam()

# Main control loop
def control_servo():
    global servo1_pos, servo2_pos, servo3_pos, servo4_pos
    current_id = 1  # Start with servo 1

    while True:
        print(f"\nCurrent Servo ID: {current_id}")
        print("Press 'w' to increase position, 's' to decrease position")
        print("Press 'a' to switch to previous servo ID, 'd' to switch to next servo ID")
        print("Press 'ESC' to exit")

        key = getch()

        # Exit on ESC
        if key == chr(0x1b):
            break

        # Increase position with 'w'
        elif key == 'w':
            if current_id == 1 and servo1_pos < SCS_MAXIMUM_POSITION_VALUE:
                servo1_pos += step
            elif current_id == 2 and servo2_pos < SCS_MAXIMUM_POSITION_VALUE:
                servo2_pos += step
            elif current_id == 3 and servo3_pos < SCS_MAXIMUM_POSITION_VALUE:
                servo3_pos += step
            elif current_id == 4 and servo4_pos < SCS_MAXIMUM_POSITION_VALUE:
                servo4_pos += step
            print(f"Increased position for Servo {current_id}")

        # Decrease position with 's'
        elif key == 's':
            if current_id == 1 and servo1_pos > SCS_MINIMUM_POSITION_VALUE:
                servo1_pos -= step
            elif current_id == 2 and servo2_pos > SCS_MINIMUM_POSITION_VALUE:
                servo2_pos -= step
            elif current_id == 3 and servo3_pos > SCS_MINIMUM_POSITION_VALUE:
                servo3_pos -= step
            elif current_id == 4 and servo4_pos > SCS_MINIMUM_POSITION_VALUE:
                servo4_pos -= step
            print(f"Decreased position for Servo {current_id}")

        # Switch to previous servo ID with 'a'
        elif key == 'a':
            if current_id > 1:
                current_id -= 1
            else:
                print("Already at Servo ID 1.")
            print(f"Switched to Servo {current_id}")

        # Switch to next servo ID with 'd'
        elif key == 'd':
            if current_id < 4:
                current_id += 1
            else:
                print("Already at Servo ID 4.")
            print(f"Switched to Servo {current_id}")

        # Update servo positions and write to servos
        sync_write_positions()

# Run the control function
control_servo()

# Close port
portHandler.closePort()
