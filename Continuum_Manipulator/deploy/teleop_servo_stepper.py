#!/usr/bin/env python3
"""
Earlier keyboard teleop for the 4 Feetech STS servos and the 4 Emm V5 stepper slides
(same key layout as collect_and_track.py, without ROS and recording).
"""
import os
import time
import serial
import struct
import sys, tty, termios
from scservo_sdk import *
###import RPi.GPIO as GPIO

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

counter = [0]*4

BAUDRATE = 1000000
DEVICENAME = '/dev/ttyCH341USB3'
SCS_MINIMUM_POSITION_VALUE = 0
SCS_MAXIMUM_POSITION_VALUE = 28672
SCS_MOVING_SPEED = 600
SCS_MOVING_ACC = 50
step = 200
step2 = 3200 #滑轨的step

servo1_pos = 2400
servo2_pos = 3312
servo3_pos = 2200
servo4_pos = 2600

portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)

uart1 = serial.Serial(port="/dev/ttyUSB0", baudrate=115200)
uart2 = serial.Serial(port="/dev/ttyCH341USB1", baudrate=115200)
uart3 = serial.Serial(port="/dev/ttyCH341USB2", baudrate=115200)
uart4 = serial.Serial(port="/dev/ttyCH341USB0", baudrate=115200)

if portHandler.openPort():
    print("Succeeded to open the port")
else:
    print("Failed to open the port")
    getch()
    quit()

if portHandler.setBaudRate(BAUDRATE):
    print("Succeeded to change the baudrate")
else:
    print("Failed to change the baudrate")
    getch()
    quit()

def sync_write_positions():
    print(f"Servo1: {servo1_pos}, Servo2: {servo2_pos}, Servo3: {servo3_pos}, Servo4: {servo4_pos}")
    packetHandler.SyncWritePosEx(1, servo1_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.SyncWritePosEx(2, servo2_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.SyncWritePosEx(3, servo3_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.SyncWritePosEx(4, servo4_pos, SCS_MOVING_SPEED, SCS_MOVING_ACC)
    packetHandler.groupSyncWrite.txPacket()
    packetHandler.groupSyncWrite.clearParam()


def Emm_V5_Pos_Control(addr, dir, vel, acc, clk, raF, snF, setted):
    global counter
    cmd = bytearray(16)
    cmd[0] = addr
    cmd[1] = 0xFD
    cmd[2] = dir
    cmd[3] = (vel >> 8) & 0xFF
    cmd[4] = vel & 0xFF
    cmd[5] = acc
    cmd[6] = (clk >> 24) & 0xFF
    cmd[7] = (clk >> 16) & 0xFF
    cmd[8] = (clk >> 8) & 0xFF
    cmd[9] = clk & 0xFF
    cmd[10] = 0x01 if raF else 0x00
    cmd[11] = 0x01 if snF else 0x00
    cmd[12] = 0x6B

    uart1.write(cmd[:13])
    uart2.write(cmd[:13])
    uart3.write(cmd[:13])
    uart4.write(cmd[:13])
    
    if setted:
        if dir:
            counter[addr-1]-=step2
            print(addr-1,counter[addr-1])
        else:
            counter[addr-1]+=step2
            print(addr-1,counter[addr-1])

def Emm_V5_Stop_Now(addr, snF): # 地址电机，不启用多机同步，立即停止
    cmd = bytearray(5)
    cmd[0] = addr               # 地址
    cmd[1] = 0xFE               # 功能码
    cmd[2] = 0x98               # 辅助码
    cmd[3] = 0x01 if snF else 0x00  # 多机同步运动标志，true为0x01，false为0x00
    cmd[4] = 0x6B               # 校验字节
    uart1.write(cmd)
    uart2.write(cmd)
    uart3.write(cmd)
    uart4.write(cmd)

def Emm_V5_Synchronous_motion(addr): # 执行地址电机多机同步运动命令
    cmd = bytearray(4)
    cmd[0] = addr               # 地址
    cmd[1] = 0xFF               # 功能码
    cmd[2] = 0x66               # 辅助码
    cmd[3] = 0x6B               # 校验字节
    uart1.write(cmd)
    uart2.write(cmd)
    uart3.write(cmd)
    uart4.write(cmd)

def Emm_V5_Pos_Recover():
    global counter
    if counter[0] > 0:
        Emm_V5_Pos_Control(1, 1, 1000, 50, counter[0], 0, True,0)
    elif counter[0] < 0:
        Emm_V5_Pos_Control(1, 0, 1000, 50, -counter[0], 0, True,0)
    time.sleep(0.1)
    if counter[1] > 0:
        Emm_V5_Pos_Control(2, 1, 1000, 50, counter[1], 0, True,0)
    elif counter[1] < 0:
        Emm_V5_Pos_Control(2, 0, 1000, 50, -counter[1], 0, True,0)
    time.sleep(0.1)
    if counter[2] > 0:
        Emm_V5_Pos_Control(3, 1, 1000, 50, counter[2], 0, True,0)
    elif counter[2] < 0:
        Emm_V5_Pos_Control(3, 0, 1000, 50, -counter[2], 0, True,0)
    time.sleep(0.1)
    if counter[3] > 0:
        Emm_V5_Pos_Control(4, 1, 1000, 50, counter[3], 0, True,0)
    elif counter[3] < 0:
        Emm_V5_Pos_Control(4, 0, 1000, 50, -counter[3], 0, True,0)
    time.sleep(0.1)
    Emm_V5_Synchronous_motion(0)
   # Emm_V5_Synchronous_motion(1)
   # Emm_V5_Synchronous_motion(2)
   # Emm_V5_Synchronous_motion(3)
   # Emm_V5_Synchronous_motion(4)

    for i in range(4):
        counter[i]=0

    
def together(pos1,pos2,pos3,pos4):
    if pos1 > 0:
        Emm_V5_Pos_Control(1, 1, 1000, 50, pos1, 0, True,0)
    elif pos1 < 0:
        Emm_V5_Pos_Control(1, 0, 1000, 50, -pos1, 0, True,0)
    time.sleep(0.1)
    if pos2 > 0:
        Emm_V5_Pos_Control(2, 1, 1000, 50, pos2, 0, True,0)
    elif pos2 < 0:
        Emm_V5_Pos_Control(2, 0, 1000, 50, -pos2, 0, True,0)
    time.sleep(0.1)
    if pos3 > 0:
        Emm_V5_Pos_Control(3, 1, 1000, 50, pos3, 0, True,0)
    elif pos3 < 0:
        Emm_V5_Pos_Control(3, 0, 1000, 50, -pos3, 0, True,0)
    time.sleep(0.1)
    if pos4 > 0:
        Emm_V5_Pos_Control(4, 1, 1000, 50, pos4, 0, True,0)
    elif pos4 < 0:
        Emm_V5_Pos_Control(4, 0, 1000, 50, -pos4, 0, True,0)
    time.sleep(0.1)
    Emm_V5_Synchronous_motion(0)

def control_devices():
    global servo1_pos, servo2_pos, servo3_pos, servo4_pos, counter, step, step2
    current_id = 1
    lever = False
    print("servo_control:")
    print("W: increase servo pos, S: decrease servo pos")
    print("A: last servo, D: next servp")
    print("step_motor_control:")
    print("e: motor1 cw, r: motor1 ccw")
    print("t: motor2 cw, y: motor2 ccw")
    print("u: motor3 cw, i: motor3 ccw")
    print("O: motor4 cw, P: motor4 ccw")
    print("ESC: exit")

    while True:
        key = getch()


        if key == chr(0x1b):
            break


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
        elif key == 'a':
            current_id = max(1, current_id - 1)
            print(f"Switched to Servo {current_id}")
        elif key == 'd':
            current_id = min(4, current_id + 1)
            print(f"Switched to Servo {current_id}")


        elif key == 'e':
            Emm_V5_Pos_Control(1, 0, 1000, 50, step2, 0, False, lever)
            print("Motor 1 CW")
        elif key == 'r':
            Emm_V5_Pos_Control(1, 1, 1000, 50, step2, 0, False, lever)
            print("Motor 1 CCW")
        elif key == 't':
            Emm_V5_Pos_Control(2, 0, 1000, 50, step2, 0, False, lever)
            print("Motor 2 CW")
        elif key == 'y':
            Emm_V5_Pos_Control(2, 1, 1000, 50, step2, 0, False, lever)
            print("Motor 2 CCW")
        elif key == 'u':
            Emm_V5_Pos_Control(3, 0, 1000, 50, step2, 0, False, lever)
            print("Motor 3 CW")
        elif key == 'i':
            Emm_V5_Pos_Control(3, 1, 1000, 50, step2, 0, False, lever)
            print("Motor 3 CCW")
        elif key == 'o':
            Emm_V5_Pos_Control(4, 0, 1000, 50, step2, 0, False, lever)
            print("Motor 4 CW")
        elif key == 'p':
            Emm_V5_Pos_Control(4, 1, 1000, 50, step2, 0, False, lever)
            print("Motor 4 CCW")
        elif key=='m':#设置零点
            lever = True
            print("Origin Setted")
        elif key == 'n':#回零
            Emm_V5_Pos_Recover()
            print("Recovering")
        elif key == 'b':
            Emm_V5_Stop_Now(0,False)
        elif key == 'g':
            together(step2,step2,step2,step2)
        elif key == 'h':
            together(-step2,-step2,-step2,-step2)
        elif key=='q':
            Emm_V5_Pos_Recover()
            print("Recovering")
            break

            
        sync_write_positions()

control_devices()

portHandler.closePort()
