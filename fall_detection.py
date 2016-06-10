#!/usr/bin/python

from __future__ import division
import time
from math import *
from multiprocessing import *
from mpu6050_process_1 import MPU6050_Process_1
from mpu6050_process_2 import MPU6050_Process_2
from pyModbusTCP.client import ModbusClient

with Manager() as manager:

    try:
        c = ModbusClient(host="140.116.82.50", port=7654)
    except ValueError:
        print("Error with host or port params")
    c.open()
    is_ok = c.write_single_register(0, 1)
    if not is_ok:
        c.open()

    sleep_time = 0.1

    T_alpha_a = 3.0
    T_alpha_b = 2.5
    T_omg_a = 200
    T_omg_b = 340

    acc_offset_1 = [50, 0, -300]
    acc_offset_2 = [120, -22.5, -345]
    acc_scale_1 = [8160, 8150, 8200]
    acc_scale_2 = [8210, 8315, 8172.5]
    _4g = 0.0001220703125

    data_1 = Array('i', 6)
    data_2 = Array('i', 6)
    lock = manager.Lock()

    mpu1 = MPU6050_Process_1(0x68, lock, data_1)
    mpu2 = MPU6050_Process_2(0x69, lock, data_2)

    mpu1.daemon = True
    mpu2.daemon = True
    mpu1.start()
    mpu2.start()

    while True:
        alpha_a_int = []
        alpha_b_int = []
        omg_a_int = []
        omg_b_int = []
        for _ in xrange(10):
            acc_1 = [data_1[0]-acc_offset_1[0], data_1[1]-acc_offset_1[1], data_1[2]-acc_offset_1[2]]
            acc_2 = [data_2[0]-acc_offset_2[0], data_2[1]-acc_offset_2[1], data_2[2]-acc_offset_2[2]]
            gyro_1 = [data_1[3], data_1[4], data_1[5]]
            gyro_2 = [data_2[3], data_2[4], data_2[5]]

            if max(gyro_1) > 600 or min(gyro_1) < -600 or max(gyro_2) > 600 or min(gyro_2) < -600:
                time.sleep(sleep_time)
                continue

            for i in xrange(3):
                acc_1[i] = acc_1[i] * _4g * acc_scale_1[i]
                acc_2[i] = acc_2[i] * _4g * acc_scale_2[i]
            alpha_a_int.append(sqrt(pow(acc_1[0],2) + pow(acc_1[1],2) + pow(acc_1[2],2)))
            alpha_b_int.append(sqrt(pow(acc_2[0],2) + pow(acc_2[1],2) + pow(acc_2[2],2)))
            omg_a_int.append(sqrt(pow(gyro_1[3],2) + pow(gyro_1[4],2) + pow(gyro_1[5],2)))
            omg_b_int.append(sqrt(pow(gyro_2[3],2) + pow(gyro_2[4],2) + pow(gyro_2[5],2)))
            time.sleep(sleep_time)

        if (max(alpha_a_int) - min(alpha_a_int) < 0.4 and max(alpha_b_int) - min(alpha_b_int) < 0.4 
                and max(omg_a_int) - min(omg_a_int) < 60 and max(omg_b_int) - min(omg_b_int) < 60):
            is_ok = c.write_single_register(1, 0)
            if not is_ok:
                c.open()

            if acos(min(acc_1[0])) > 35 and acos(min(acc_2[0])) > 35:
                if (max(alpha_a_int) > T_alpha_a and max(alpha_b_int) > T_alpha_b 
                        and max(omg_a_int) > T_omg_a and max(omg_b_int) > T_omg_b):
                    is_ok = c.write_single_register(2, 1)
                    if not is_ok:
                        c.open()

        else:
            is_ok = c.write_single_register(1, 1)
            if not is_ok:
                c.open()
