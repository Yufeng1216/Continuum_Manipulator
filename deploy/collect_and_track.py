#!/usr/bin/env python3
"""
Main control script for the MSESM, run on the robot PC (Ubuntu 20.04 + ROS Noetic).

- Thread 1 subscribes to /ar_pose_marker (ar_track_alvar) and keeps the latest AR-tag pose.
- Thread 2 is a keyboard teleop for the 4 Feetech servos (C2 tendons) and the 4 Emm V5
  stepper slides (C1 Teflon tubes), and sends target pressures to the Raspberry Pi over UART.
  Data rows (actuator state + AR pose) are appended to a CSV for training.
- Key 'f' runs start(): an ONNX network predicts servo positions, pressure and PCC
  parameters (L, alpha, theta) for each waypoint of a circle, and calculate_lengths()
  converts (L, alpha, theta) into tendon lengths / stepper pulses.

Known issue: in start(), `new_data[0] = x` and `new_data[2] = y` should be
`new_data[0][0] = x` and `new_data[0][2] = y` (new_data has shape 1 x 6).
"""
import os
import time
import serial
import struct
import random
import sys, tty, termios
from scservo_sdk import *
import select
import math
import csv
from threading import Thread
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
import onnxruntime as ort
import joblib
import numpy as np

###import RPi.GPIO as GPIO
csv_file_path = os.path.expanduser("~/stepservo_ws/src/motor/scripts/data1.csv")
counter = [0]*4
L = 0
alpha = 0
theta = 0
r = 6.9
r2 = 4.95
BAUDRATE = 1000000
DEVICENAME = '/dev/ttyUSB0'
SCS_MINIMUM_POSITION_VALUE = 0
SCS_MAXIMUM_POSITION_VALUE = 28672
SCS_MOVING_SPEED = 900
SCS_MOVING_ACC = 100
step = 200
step2 = 6400 #滑轨的step
total_pulse_moved = [0, 0, 0, 0]
servo1_pos_org = 48
servo2_pos_org = 1262
servo3_pos_org = 2440
servo4_pos_org = 2944
servo1_pos = servo1_pos_org
servo2_pos = servo2_pos_org
servo3_pos = servo3_pos_org
servo4_pos = servo4_pos_org
portHandler = PortHandler(DEVICENAME)
packetHandler = sms_sts(portHandler)
ar_pose_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
new_data = np.array([[0.95, 0, -0.1,
                      90, 0, -90]])
press1=110000
press2=103000
uart1 = serial.Serial(port="/dev/ttyCH9344USB0", baudrate=115200)
uart2 = serial.Serial(port="/dev/ttyCH9344USB1", baudrate=115200)
uart3 = serial.Serial(port="/dev/ttyCH9344USB2", baudrate=115200)
uart4 = serial.Serial(port="/dev/ttyCH9344USB3", baudrate=115200)
uart5 = serial.Serial(port="/dev/ttyCH9344USB6", baudrate=9600) #和树莓派通信用的uart

def pose_callback(data):
    global ar_pose_data  # 声明全局变量
    if len(data.markers) > 0:
        # 假设只使用第一个 marker 的数据
        marker = data.markers[0]
        position = marker.pose.pose.position
        orientation = marker.pose.pose.orientation

        
        ar_pose_data = [
            position.x, position.y, position.z,
            orientation.x, orientation.y, orientation.z, orientation.w
        ]
                                       
                     

def listener():
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, pose_callback)
    rospy.spin()


def append_to_csv(data, headers):
    file_exists = os.path.isfile(csv_file_path)
    with open(csv_file_path, mode='a', newline='', encoding='utf-8') as file:
        writer = csv.DictWriter(file, fieldnames=headers)
        if not file_exists:
            writer.writeheader()
        writer.writerow(data)
      
def calculate_lengths(L, r, theta, alpha, flag):
    lengths = []
    proportion = 407
    for j in range(1, 5):
        beta_j = alpha + (j - 1) * (math.pi / 2)
        l_j = 2 * (L / theta) * math.sin(theta / 2) - 2 * r * math.cos(beta_j) * math.sin(theta / 2)
        lengths.append(l_j)
    if flag == 1:
        print(f"lengths:{lengths}\n")
        return -int((26-lengths[0])*8000),-int((26-lengths[1])*8000),-int((26-lengths[2])*8000),-int((26-lengths[3])*8000)
    if flag == 2:
        
        print(f"lengths:{lengths}\n")
        return int((30.2-lengths[0])*proportion),int((30.2-lengths[1])*proportion),int((30.2-lengths[2])*proportion),int((30.2-lengths[3])*proportion)


 
def record_data():
    global servo1_pos, servo2_pos, servo3_pos, servo4_pos, total_pulse_moved, press1, press2, ar_pose_data 

    headers = [
               'Servo1_Pos', 'Servo2_Pos', 'Servo3_Pos', 'Servo4_Pos',
               'Step_Motor1', 'Step_Motor2', 'Step_Motor3', 'Step_Motor4',
               'Pressure_1', 'Pressure_2',
                'AR_Pose_X', 'AR_Pose_Y', 'AR_Pose_Z',
               'AR_Orientation_X', 'AR_Orientation_Y', 'AR_Orientation_Z', 'AR_Orientation_W']
    
    if not ar_pose_data:
        print("No topic data detected. Exiting program.")
        rospy.signal_shutdown("No topic data detected")
        sys.exit(1)  

    else:       
        data = {
            'Servo1_Pos': servo1_pos,
            'Servo2_Pos': servo2_pos,
            'Servo3_Pos': servo3_pos,
            'Servo4_Pos': servo4_pos,
            'Step_Motor1': total_pulse_moved[0],
            'Step_Motor2': total_pulse_moved[1],
            'Step_Motor3': total_pulse_moved[2],
            'Step_Motor4': total_pulse_moved[3],
            'Pressure_1': press1,
            'Pressure_2': press2,
            'AR_Pose_X': ar_pose_data[0],
            'AR_Pose_Y': ar_pose_data[1],
            'AR_Pose_Z': ar_pose_data[2],
            'AR_Orientation_X': ar_pose_data[3],
            'AR_Orientation_Y': ar_pose_data[4],
            'AR_Orientation_Z': ar_pose_data[5],
            'AR_Orientation_W': ar_pose_data[6]
        }
        append_to_csv(data, headers)
    print("record")
   
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

def send_data_to_uart(uart, set_p1, set_p2):
    try:
        # 将数据格式化成字符串，使用逗号分隔，最后加上换行符
        data = f"{set_p1},{set_p2}"
        
        # 发送数据到 UART
        uart.write(data.encode('utf-8'))
        
        print(f"Data sent to UART: {data.strip()}")
    except Exception as e:
        print(f"Error sending data to UART: {e}")

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
            counter[addr-1]-=clk
            print(addr-1,counter[addr-1])
        else:
            counter[addr-1]+=clk
            print(addr-1,counter[addr-1])
    time.sleep(0.01)

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
    global counter
    if pos1<counter[0]:
        Emm_V5_Pos_Control(1, 1, 500, 25, counter[0]-pos1, 0, False, 1)
    elif pos1>counter[0]:
        Emm_V5_Pos_Control(1, 0, 500, 25, pos1-counter[0], 0, False, 1)
    if pos2<counter[1]:
        Emm_V5_Pos_Control(2, 1, 500, 25, counter[1]-pos2, 0, False, 1)
    elif pos2>counter[1]:
        Emm_V5_Pos_Control(2, 0, 500, 25, pos2-counter[1], 0, False, 1)
    if pos3<counter[2]:
        Emm_V5_Pos_Control(3, 1, 500, 25, counter[2]-pos3, 0, False, 1)
    elif pos3>counter[2]:
        Emm_V5_Pos_Control(3, 0, 500, 25, pos3-counter[2], 0, False, 1)
    if pos4<counter[3]:
        Emm_V5_Pos_Control(4, 1, 500, 25, counter[3]-pos4, 0, False, 1)
    elif pos4>counter[3]:
        Emm_V5_Pos_Control(4, 0, 500, 25, pos4-counter[3], 0, False, 1)
    time.sleep(0.1)


def start():
    global servo1_pos,servo2_pos,servo3_pos,servo4_pos,counter,r,total_pulse_moved, press1, press2
    flag=0
    ort_session = ort.InferenceSession("models/my_model_euler.onnx")
    print("模型已成功加载！")
    scaler_X = joblib.load('models/best_scaler_X_euler.pkl')
    scaler_y = joblib.load('models/best_scaler_y_euler.pkl')
    # 圆心坐标
    center_x = 0.95
    center_y = -0.1
    # 圆的半径
    radius = 0.1
    # 生成点的数量
    num_points = 100
    # 生成角度（从0到2π均匀分布）
    angles = np.linspace(0, 2 * np.pi, num_points)
    # 计算圆上点的坐标
    x_points = center_x + radius * np.cos(angles)
    y_points = center_y + radius * np.sin(angles)
    for x, y in zip(x_points, y_points):
        new_data[0]=x
        new_data[2]=y
        new_data_scaled = scaler_X.transform(new_data)
        ort_inputs = {ort_session.get_inputs()[0].name: new_data_scaled.astype(np.float32)}
        ort_outs = ort_session.run(None, ort_inputs)
        predictions = scaler_y.inverse_transform(ort_outs[0])
        print(f"prediction:{predictions}")
        L=predictions[0][5]
        alpha=predictions[0][6]
        theta=predictions[0][7]
        total_pulse_moved=calculate_lengths(L,6.9,theta,alpha,1)
        servo1_pos=predictions[0][0]
        servo2_pos=predictions[0][1]
        servo3_pos=predictions[0][2]
        servo4_pos=predictions[0][3]
        press2=predictions[0][4]
        together(total_pulse_moved[0],total_pulse_moved[1],total_pulse_moved[2],total_pulse_moved[3])
        sync_write_positions()
        send_data_to_uart(uart5,press1,press2)
        if flag == 0:
            time.sleep(15)
            flag=1
        else:
            time.sleep(5)


def getch_timeout(timeout_seconds):
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        rlist, _, _ = select.select([sys.stdin], [], [], timeout_seconds)
        
        if rlist:
            char = sys.stdin.read(1)  # 获取输入的字符
            return char
        else:
            return None  # 超时返回 None
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)



def control_devices():
    global servo1_pos, servo2_pos, servo3_pos, servo4_pos, counter, step, step2, press1, press2
    current_id = 1
    lever = False
    lever2 = False
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
        
        key = getch_timeout(100)
        if key == chr(0x1b):
            break
        if key is None:
            print('Time out')            

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
        elif key == 'f':
            start()
        elif key=='q':
            Emm_V5_Pos_Recover()
            packetHandler.SyncWritePosEx(1, servo1_pos_org, SCS_MOVING_SPEED, SCS_MOVING_ACC)
            packetHandler.SyncWritePosEx(2, servo2_pos_org, SCS_MOVING_SPEED, SCS_MOVING_ACC)
            packetHandler.SyncWritePosEx(3, servo3_pos_org, SCS_MOVING_SPEED, SCS_MOVING_ACC)
            packetHandler.SyncWritePosEx(4, servo4_pos_org, SCS_MOVING_SPEED, SCS_MOVING_ACC)
            packetHandler.groupSyncWrite.txPacket()
            packetHandler.groupSyncWrite.clearParam()
            send_data_to_uart(uart5,101000,103000)
            print("Recovering")
            break
        elif key == 'z':
            print("Input the pressure of chamber 1 and chamber 2")
            press1=input()
            press2=input()
            send_data_to_uart(uart5,press1,press2)
        sync_write_positions()
        print("stand by")


def ros():
    try:
        while True:
            print(f"working...")
            listener()
            time.sleep(0.05)
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    rospy.init_node('data_recorder', anonymous=True)  
    thread1 = Thread(target=ros)  
    thread2 = Thread(target=control_devices)

    thread1.start()
    thread2.start()

    thread1.join()
    thread2.join()

    portHandler.closePort()

