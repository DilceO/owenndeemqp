# Owen Rouse - 11/18/2023
# Robo Control

import os
from dynamixel_sdk import *

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

# Servo Specs
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 1000000

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

PROTOCOL_VERSION            = 2.0

# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)


#  My code

class Servo():

    def __init__(self, data,EnableTorque = True):
        j_num = data[0]
        min_rom = data[1]
        max_rom = data[2]

        self.j_num = j_num + 11
        self.min_rom = min_rom
        self.max_rom = max_rom

        # Enable Dynamixel Torque
        if EnableTorque:
            dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, self.j_num, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
            elif dxl_error != 0:
                print("%s" % packetHandler.getRxPacketError(dxl_error))
            else:
                print("Dynamixel " + str(j_num) + " has been successfully connected")
        else: print("Torque Not Enabled")

    def readAngle(self):
        # Read present position
        dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, self.j_num, ADDR_PRESENT_POSITION)

        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))
        self.angle = dxl_present_position/DXL_MAXIMUM_POSITION_VALUE * 360 - 90

        return self.angle

    def writeAngle(self,angle_deg): # Write angle from 0
        #print("Servo " + str(self.j_num) + " at " + str(angle_deg))
        angle_deg += 90
        if angle_deg > self.max_rom:
            print("Servo " + str(self.j_num) + " Exceeded Max Angle at: " + str(angle_deg))
            angle_deg = self.max_rom
        elif angle_deg < self.min_rom:
            print("Servo " + str(self.j_num) + " Exceeded Min Angle at: " + str(angle_deg))
            angle_deg = self.min_rom

        angle = int(angle_deg * (DXL_MAXIMUM_POSITION_VALUE - DXL_MINIMUM_POSITION_VALUE) / 360)

        # Write goal position
        dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, self.j_num, ADDR_GOAL_POSITION, angle)
        
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

def start():
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

def end(j_max):
    j_max = j_max+10
    # Disable Dynamixel Torque
    print("Grab Robot Arm to keep from Crashing! Press Key to continue...")
    getch()
    for i in range(0,j_max+2):
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, i, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % packetHandler.getRxPacketError(dxl_error))

    # Close port
    portHandler.closePort()        

