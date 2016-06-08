#!/usr/bin/python

import time
from multiprocessing import *
#from test import TEST
from mpu6050_process_1 import MPU6050_Process_1
from mpu6050_process_2 import MPU6050_Process_2
from pyModbusTCP.client import ModbusClient

with Manager() as manager:

    try:
        c = ModbusClient(host="140.116.82.50", port=7654)
    except ValueError:
        print("Error with host or port params")
    c.open()

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
        print('1 ', data_1[3], data_1[4], data_1[5])
        print('2 ', data_2[3], data_2[4], data_2[5])
        print(' ')
        if c.is_open():
            is_ok = c.write_single_register(0, 10)
        else:
            c.open()
        time.sleep(1)
        pass
