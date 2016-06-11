#!/usr/bin/python

import time
import math
import mpu6050_1
from multiprocessing import Process, Manager

class MPU6050_Process_1(Process):

    def __init__(self, address, lock, data):
        Process.__init__(self)
        self.addr = address
        self.lock = lock
        self.data = data
        # Sensor initialization
        self.mpu = mpu6050_1.MPU6050(self.addr)
        self.mpu.dmpInitialize()
        self.mpu.setDMPEnabled(True)
        # get expected DMP packet size for later comparison
        self.packetSize = self.mpu.dmpGetFIFOPacketSize()

    def run(self):
        while True:
            # Get INT_STATUS byte
            mpuIntStatus = self.mpu.getIntStatus()
            fifoCount = self.mpu.getFIFOCount()

            # check for DMP data ready interrupt
            if (mpuIntStatus & 0x10) or fifoCount == 1024:
                self.mpu.resetFIFO()
                #print('FIFO overflow!')
            elif mpuIntStatus & 0x02:
                # wait for correct available data length
                while fifoCount < self.packetSize:
                    fifoCount = self.mpu.getFIFOCount()
                fifoCount -= self.packetSize

                result = self.mpu.getFIFOBytes(self.packetSize)

                self.lock.acquire()
                self.mpu.dmpGetAccGyro(result, self.data)
                self.lock.release()
            time.sleep(0.00833)
                #print(self.addr, self.data)
