import numpy as np
import serial
import time
import cv2 as cv
import matplotlib.pyplot as plt

# set param of serial port
try:
    ser = serial.Serial('COM10', 115200)
    ser.bytesize = serial.EIGHTBITS  # 8位数据位
    ser.parity = serial.PARITY_NONE  # 无校验位
    ser.stopbits = serial.STOPBITS_ONE  # 1位停止位
    print("ser opened successfully!")
except:
    print("ERROR, open serial port failed")
    exit()


read_len, write_len = 8, 10
pressure, torque, angle = 0, 0, 0

# crc16
crc_talbel = [
    0x0000, 0xCC01, 0xD801, 0x1400, 0xF001, 0x3C00, 0x2800, 0xE401,
    0xA001, 0x6C00, 0x7800, 0xB401, 0x5000, 0x9C01, 0x8801, 0x4400]


def crc_16_check(data, length):
    crc = 0xFFFF
    e = 0
    for i in range(0, length):
        e = data[i]
        crc = (crc >> 4) ^ crc_talbel[(e ^ crc) & 0x0F]
        crc = (crc >> 4) ^ crc_talbel[(crc ^ (e >> 4)) & 0x0F]
    return crc


# data init
show_len = 500
start_t = time.time()
t, pressure_list, torque_list, angle_list = [0]*(show_len+1), [0]*(show_len+1), [0]*(show_len+1), [0]*(show_len+1)

# visulization init
x = np.array(t[-show_len:])
y0 = np.array(pressure_list[-show_len:])
y1 = np.array(torque_list[-show_len:])
y2 = np.array(angle_list[-show_len:])

fig, axs = plt.subplots(3, 1, figsize=(6, 4))
axs[0].plot(x, y0, label='pressure')
axs[1].plot(x, y1, label='torque')
axs[2].plot(x, y2, label='angle')

while True:
    if ser.in_waiting <= 0:
        time.sleep(0.001)
    else:
        pfa_data = ser.read(read_len)
        ser.flushInput()
        crc_16_result = crc_16_check(pfa_data, read_len-2)
        print(crc_16_result, int.from_bytes(pfa_data[-2:], byteorder='big'))
        if crc_16_result == int.from_bytes(pfa_data[-2:], byteorder='big'):
            pressure = int.from_bytes(pfa_data[0:2], byteorder='big', signed=False)
            torque = int.from_bytes(pfa_data[2:4], byteorder='big', signed=False)
            angle = int.from_bytes(pfa_data[4:6], byteorder='big', signed=False)
            pressure_list.append(pressure)
            torque_list.append(torque)
            angle_list.append(angle)
            t.append(time.time() - start_t)
            print("p:", pfa_data[0:2], ", t:", pfa_data[2:4], ", a:", pfa_data[4:6])
        else:
            print("crc error")
            pressure_list.append(pressure)
            torque_list.append(torque)
            angle_list.append(angle)
            t.append(time.time() - start_t)

        axs[0].cla()
        axs[1].cla()
        axs[2].cla()
        x = np.array(t[-show_len:])
        y0 = np.array(pressure_list[-show_len:])
        y1 = np.array(torque_list[-show_len:])
        y2 = np.array(angle_list[-show_len:])
        axs[0].plot(x, y0, label='pressure')
        axs[1].plot(x, y1, label='torque')
        axs[2].plot(x, y2, label='angle')
        axs[0].set_ylim(-100, 7000)
        axs[1].set_ylim(-100, 7000)
        axs[2].set_ylim(-10, 360)
        axs[0].legend()
        axs[1].legend()
        axs[2].legend()

        plt.draw()
        plt.pause(0.001)


ser.close()
plt.ioff()
plt.show()


import numpy as np
import matplotlib.pyplot as plt
from scipy.linalg import block_diag
from filterpy.kalman import KalmanFilter
#
# def imu_kalman_filter(imu_data):
#     n = len(imu_data)
#     dt = 1.0/60  # sample rate
#
#     kf = KalmanFilter(dim_x=3, dim_z=1) # 初始化卡尔曼滤波器
#     kf.F = np.array([[1, dt, 0.5*dt**2],
#                      [0, 1, dt],
#                      [0, 0, 1]])  # 状态过渡矩阵
#     kf.H = np.array([[1, 0, 0]])  # 评估函数
#     kf.R = np.eye(1) * 0.1
#     kf.Q = block_diag(np.eye(3)*0.1)  # 协方差矩阵
#
#     kf.x = np.zeros(3)
#     filtered_data = np.zeros((n, 3))  # 对IMU数据应用滤波器
#     for i in range(n):
#         kf.predict()
#         kf.update(imu_data[i])
#         filtered_data[i] = kf.x
#
#     return filtered_data
#
# # Simulate IMU data
# np.random.seed(0)
# imu_data = np.cumsum(np.random.randn(1000))  # IMU data: random walk
#
# # Apply Kalman filter
# filtered_data = imu_kalman_filter(imu_data)
#
# # Plot results
# plt.figure()
# plt.plot(imu_data, label='IMU')
# plt.plot(filtered_data[:, 0], label='Filtered')
# plt.legend()
# plt.show()
