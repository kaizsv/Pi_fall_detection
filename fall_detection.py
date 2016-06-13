#!/usr/bin/python

from __future__ import division
import atexit
import time
import sys
from math import *
from multiprocessing import *
from mpu6050_process_1 import MPU6050_Process_1
from mpu6050_process_2 import MPU6050_Process_2
from pyModbusTCP.client import ModbusClient

with Manager() as manager:

    # Open a modbus client
    try:
        c = ModbusClient(host="140.116.82.50", port=7654)
    except ValueError:
        print("Error with host or port params")
    c.open()
    is_ok = c.write_single_register(0, 1)
    if not is_ok:
        c.open()

    def exit_handler():
        is_ok = c.write_multiple_registers(0, [0, 0, 0, 0, 0])
        c.close()

    # registing exit handler
    atexit.register(exit_handler)

    # data sampling period
    interval = 120
    sleep_time = 1 / interval
    i = 0

    # consitent define in paper
    T_alpha_a = 2.0
    T_alpha_b = 1.5
    T_omg_a = 200
    T_omg_b = 340

    """
        Accelremeter Calibration

           max   |   min   |   offset   |   1g_scale
      x_1  8210     -8110        50           8160
      y_1  8150     -8150         0           8150
      z_1  7900     -8500      -300           8200

      x_2  8330     -8090       120           8210
      y_2  8150     -8195       -22.5         8172.5
      z_2  7970     -8660      -345           8315

      _4g = 1 / 8192

    """
    acc_offset_1 = [50, 0, -300]
    acc_offset_2 = [120, -22.5, -345]
    acc_scale_1 = [8192/8160, 8192/8150, 8192/8200]
    acc_scale_2 = [8192/8210, 8192/8315, 8192/8172.5]
    _4g = 0.0001220703125

    alpha_a_int = []
    alpha_b_int = []
    omg_a_int = []
    omg_b_int = []

    # share data for multiprocessing
    data_1 = Array('i', 6)
    data_2 = Array('i', 6)
    lock = manager.Lock()

    # start processing at address 0x68 and 0x69
    mpu1 = MPU6050_Process_1(0x68, lock, data_1)
    mpu2 = MPU6050_Process_2(0x69, lock, data_2)

    mpu1.daemon = True
    mpu2.daemon = True
    mpu1.start()
    mpu2.start()

    # init list
    for _ in xrange(interval):
        alpha_a_int.append(0)
        alpha_b_int.append(0)
        omg_a_int.append(0)
        omg_b_int.append(0)

    while True:

        acc_1 = [data_1[0]-acc_offset_1[0], data_1[1]-acc_offset_1[1], data_1[2]-acc_offset_1[2]]
        acc_2 = [data_2[0]-acc_offset_2[0], data_2[1]-acc_offset_2[1], data_2[2]-acc_offset_2[2]]
        gyro_1 = [data_1[3], data_1[4], data_1[5]]
        gyro_2 = [data_2[3], data_2[4], data_2[5]]

        # skip noise data
        if max(gyro_1) > 600 or min(gyro_1) < -600 or max(gyro_2) > 600 or min(gyro_2) < -600:
            time.sleep(sleep_time)
            continue

        # scale acc data
        for j in xrange(3):
            acc_1[j] = acc_1[j] * _4g * acc_scale_1[j]
            acc_2[j] = acc_2[j] * _4g * acc_scale_2[j]
        #print(acc_1, acc_2, gyro_1, gyro_2)
        alpha_a_int[i] = sqrt(pow(acc_1[0],2) + pow(acc_1[1],2) + pow(acc_1[2],2))
        alpha_b_int[i] = sqrt(pow(acc_2[0],2) + pow(acc_2[1],2) + pow(acc_2[2],2))
        omg_a_int[i] = sqrt(pow(gyro_1[0],2) + pow(gyro_1[1],2) + pow(gyro_1[2],2))
        omg_b_int[i] = sqrt(pow(gyro_2[0],2) + pow(gyro_2[1],2) + pow(gyro_2[2],2))

        """
            Monitor if people are static or dynamic during the
            present time segment.
        """
        if (max(alpha_a_int) - min(alpha_a_int) < 0.45 and max(alpha_b_int) - min(alpha_b_int) < 0.45 
                and max(omg_a_int) - min(omg_a_int) < 70 and max(omg_b_int) - min(omg_b_int) < 70):
            # static
            is_ok = c.write_single_register(1, 0)
            if not is_ok:
                c.open()

            """
                Recognize the present static posture: is it lying?
            """
            acc_1[0] = max(-1, min(1, acc_1[0]))
            acc_2[0] = max(-1, min(1, acc_2[0]))
            if acos(acc_1[0]) > 0.61 and acos(acc_2[0]) > 0.61:
                is_ok = c.write_single_register(3, 1)
                if not is_ok:
                    c.open()
                """
                    Determine if the transition before the present
                    lying posture is intentional.
                """
                if (max(alpha_a_int) > T_alpha_a or max(alpha_b_int) > T_alpha_b 
                        or max(omg_a_int) > T_omg_a or max(omg_b_int) > T_omg_b):
                    # fall
                    is_ok = c.write_single_register(2, 1)
                    if not is_ok:
                        c.open()
                    time.sleep(5)
                    # reset fall
                    is_ok = c.write_single_register(2, 0)
                    if not is_ok:
                        c.open()
            else:
                is_ok = c.write_single_register(3, 0)
                if not is_ok:
                    c.open()


        else:
            # dynamic
            is_ok = c.write_single_register(1, 1)
            if not is_ok:
               c.open()

        reg = c.read_holding_registers(4, 1)
        if reg[0] == 2:
            sys.exit()

        f = open('C.txt', 'a')
        str1 = ''.join(str(alpha_a_int[i]))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(alpha_b_int[i]))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(omg_a_int[i]))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(omg_b_int[i]))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(c.read_holding_registers(1, 1)))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(c.read_holding_registers(3, 1)))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(c.read_holding_registers(2, 1)))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(max(alpha_a_int) - min(alpha_a_int)))
        f.write(str1)
        f.write(' ')
        str1 = ''.join(str(max(alpha_b_int) - min(alpha_b_int)))
        f.write(str1)
        f.write('\n')
        f.close()

        i += 1
        i %= interval
        time.sleep(sleep_time)
