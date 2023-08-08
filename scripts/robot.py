#!/usr/bin/env python3
# -*- coding: utf-8 -*-

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

from dynamixel_sdk import *


MY_DXL = 'XL320'

ADDR_TORQUE_ENABLE          = 24
ADDR_TORQUE_MAX    = 15
ADDR_GOAL_POSITION          = 30
ADDR_PRESENT_POSITION       = 37
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 980     # Refer to the CCW Angle Limit of product eManual
BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

ADDR_RETURN_DELAY_TIME = 5
DELAY_TIME = 0 

PROTOCOL_VERSION            = 2.0



DXL_ID_LEFT_X                      = 1
DXL_ID_LEFT_Y                     = 10
DXL_ID_RIGHT_X                      = 6
DXL_ID_RIGHT_Y                      = 5


# left_x, left_y, right_x, right_y
ids = [DXL_ID_LEFT_X, DXL_ID_LEFT_Y, DXL_ID_RIGHT_X, DXL_ID_RIGHT_Y]

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyACM0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
TORQUE_MAX               = 1023


class Robot:

    def __init__(self):
        self.portHandler = PortHandler(DEVICENAME)

        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        self.openPort()
        self.setBaudrate()
        self.enableTorque()
        self.setDelayTime()

    def openPort(self):
         # Open port
        try:
            self.portHandler.openPort()
            print("Succeeded to open the port")
        except:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

    def setDelayTime(self):
        for id in ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_RETURN_DELAY_TIME, DELAY_TIME)
            while dxl_comm_result != COMM_SUCCESS:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_RETURN_DELAY_TIME, DELAY_TIME)
                print("Error with ", id)
    
    def setBaudrate(self):
        try:
            self.portHandler.setBaudRate(BAUDRATE)
            print("Succeeded to change the baudrate")
        except:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def enableTorque(self):
        for id in ids:
            dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            while dxl_comm_result != COMM_SUCCESS:
                dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
                print("Error with ", id)

            dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_TORQUE_MAX, TORQUE_MAX)
            while dxl_comm_result != COMM_SUCCESS:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, id, ADDR_TORQUE_MAX, TORQUE_MAX)
                print("Error with ", id)
            print("DYNAMIXEL has been successfully connected")
            
            

        print("Ready to get & set Position.")

    
    def setPositions(self, positions):

        # print("Goal: " + str(positions[0]))

        # for i in range(4):
        #     while 1:
        #         dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[i], ADDR_GOAL_POSITION, clip(positions[i]))
        #         if dxl_comm_result == COMM_SUCCESS:
        #             break

        
        if(inRange(positions[0])and inRange(positions[2])):
            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[0], ADDR_GOAL_POSITION, clip(positions[0]))
                if dxl_comm_result == COMM_SUCCESS:
                    break

            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[2], ADDR_GOAL_POSITION, clip(positions[2]))
                if dxl_comm_result == COMM_SUCCESS:
                    break


        if(inRange(positions[1])and inRange(positions[3])):
            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[1], ADDR_GOAL_POSITION, clip(positions[1]))
                if dxl_comm_result == COMM_SUCCESS:
                    break

            while 1:
                dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(self.portHandler, ids[3], ADDR_GOAL_POSITION, clip(positions[3]))
                if dxl_comm_result == COMM_SUCCESS:
                    break

    def incrementPositions(self, positions):
        pos = []

        currentPos = self.getPositions()

        for i in range(4):
            pos.append(positions[i] + currentPos[i])

        self.setPositions(pos)

    def getPositions(self):
        positions = []

        for i in range(4):
            while 1:
                dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read2ByteTxRx(self.portHandler, ids[i], ADDR_PRESENT_POSITION)
                if dxl_comm_result == COMM_SUCCESS:
                    break

            positions.append(dxl_present_position)

        
        return positions




def clip(position):
    if position>DXL_MAXIMUM_POSITION_VALUE:
        return DXL_MAXIMUM_POSITION_VALUE
    elif position<DXL_MINIMUM_POSITION_VALUE:
        return DXL_MINIMUM_POSITION_VALUE
    else:
        return round(position)
    
def inRange(position):
    return DXL_MINIMUM_POSITION_VALUE<= position and position<=DXL_MAXIMUM_POSITION_VALUE