# (c) 2023 Robotics Engineering Department, WPI
# Class for OpenManipulator-X arm
# Used to abstract serial connection and read/write methods from Robot class
# This class/file should not need to be modified in any way for RBE 3001
import ctypes
import serial
import DX_XM430_W350.py as DX

class OM_X_arm:
    def __init__(self):
        # DX_XM430_W350 Servos
        self.motor2_offset = -45 # Example offset, adjust as needed
        self.motor3_offset = -48
        self.motorsNum = None
        self.motorIDs = None
        self.gripperID = None
        self.motors = [] # 4 Motors, joints 1-4
        self.gripper = None # Gripper end-effector
        # Serial Communication variables
        self.port_num = None
        self.groupwrite_num = None
        self.groupread_num = None
        self.deviceName = None

        # Load Libraries
        DX_XM430_W350 = ctypes.CDLL("DX_XM430_W350")
        self.motorsNum = 4
        self.motorIDs = [11, 12, 13, 14]
        self.gripperID = 15
        # Find serial port and connect to it
        try:
            devices = serial.tools.list_ports.comports()
            ttyDevs = [dev.device for dev in devices if "/dev/ttyUSB" in dev.device]
            self.deviceName = ttyDevs[0]
        except Exception as e:
            raise Exception("Failed to connect via serial, no devices found.")
        self.port_num = DX_XM430_W350.portHandler(self.deviceName.encode())
        # Open port
        if not DX_XM430_W350.openPort(self.port_num):
            DX_XM430_W350.unloadlibrary(DX_XM430_W350.LIB_NAME)
            print('Failed to open the port!\n')
            input('Press any key to terminate...\n')
            return
        # Set port baudrate
        if not DX_XM430_W350.setBaudRate(self.port_num, DX_XM430_W350.BAUDRATE):
            DX_XM430_W350.unloadlibrary(DX_XM430_W350.LIB_NAME)
            print('Failed to change the baudrate!\n')
            input('Press any key to terminate...\n')
            return
        self.groupwrite_num = DX_XM430_W350.groupBulkWrite(self.port_num, DX_XM430_W350.PROTOCOL_VERSION)
        self.groupread_num = DX_XM430_W350.groupBulkRead(self.port_num, DX_XM430_W350.PROTOCOL_VERSION)
        # Create array of motors
        for i in range(self.motorsNum):
            self.motors.append(DX_XM430_W350(self.motorIDs[i], self.deviceName.encode()))
        # Create Gripper and set operating mode/torque
        self.gripper = DX_XM430_W350(self.gripperID, self.deviceName.encode())
        self.gripper.setOperatingMode('p')
        self.gripper.toggleTorque(True)
        enable = 1
        disable = 0
        self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, disable)
        self.bulkReadWrite(DX_XM430_W350.DRIVE_MODE_LEN, DX_XM430_W350.DRIVE_MODE, DX_XM430_W350.TIME_PROF)
        self.bulkReadWrite(DX_XM430_W350.TORQUE_ENABLE_LEN, DX_XM430_W350.TORQUE_ENABLE, enable)

    def bulkReadWrite(self, n, addr, msgs=None):
        if msgs is None: # Bulk Read if msgs does not exist
            DX.DX_XM430_W350.groupBulkReadClearParam(self.groupread_num)
            result = [0] * 4
            for id in self.motorIDs:
                dxl_addparam_result = DX.DX_XM430_W350.groupBulkReadAddParam(self.groupread_num, id, addr, n)
                if dxl_addparam_result != True:
                    print('[ID:%03d ADDR:%d] groupBulkRead addparam failed\n', id, addr)
                    return
            DX.DX_XM430_W350.groupBulkReadTxRxPacket(self.groupread_num)
            dxl_comm_result = DX.DX_XM430_W350.getLastTxRxResult(self.port_num, DX.DX_XM430_W350.PROTOCOL_VERSION)
            if dxl_comm_result != DX.DX_XM430_W350.COMM_SUCCESS:
                print('%s\n', DX.DX_XM430_W350.getTxRxResult(DX.DX_XM430_W350.PROTOCOL_VERSION, dxl_comm_result))
            for i in range(self.motorsNum):
                id = self.motorIDs[i]
                dxl_getdata_result = DX.DX_XM430_W350.groupBulkReadIsAvailable(self.groupread_num, id, addr, n)
                if dxl_getdata_result != True:
                    print('[ID:%03d ADDR:%d] groupBulkRead getdata failed\n', id, addr)
                    return
                readBits = DX.DX_XM430_W350.groupBulkReadGetData(self.groupread_num, id, addr, n)
                if n == 1:
                    result[i] = ctypes.cast(ctypes.pointer(ctypes.c_uint8(readBits))), ctypes.POINTER(ctypes.c_int8).contents.value
                elif n == 2:
                    result[i] = ctypes.cast(ctypes.pointer(ctypes.c_uint16(readBits))), ctypes.POINTER(ctypes.c_int16).contents.value
                elif n == 4:
                    result[i] = ctypes.cast(ctypes.pointer(ctypes.c_uint32(readBits))), ctypes.POINTER(ctypes.c_int32).contents.value
                else:
                    raise Exception("'%s' is not a valid number of bytes to read.\n", n)
                # Subtract offsets after reading for motors 2 and 3
                if id == self.motorIDs[1]:
                    result[i] = result[i] - self.motor2_offset
                elif id == self.motorIDs[2]:
                    result[i] = result[i] - self.motor3_offset
            return result
        else: # Bulk Write if msgs exists
            if len(msgs) == 1:
                msgs = [msgs] * 4
            # Add offset to motor 3 joint angle before writing
            msgs[1] = msgs[1] + self.motor2_offset
            msgs[2] = msgs[2] + self.motor3_offset
            DX.DX_XM430_W350.groupBulkWriteClearParam(self.groupwrite_num)
            for i in range(len(self.motorIDs)):
                id = self.motorIDs[i]
                if n == 1:
                    msg = ctypes.cast(ctypes.pointer(ctypes.c_int8(msgs[i]))), ctypes.POINTER(ctypes.c_uint8).contents.value
                elif n == 2:
                    msg = ctypes.cast(ctypes.pointer(ctypes.c_int16(msgs[i]))), ctypes.POINTER(ctypes.c_uint16).contents.value
                elif n == 4:
                    msg = ctypes.cast(ctypes.pointer(ctypes.c_int32(msgs[i]))), ctypes.POINTER(ctypes.c_uint32).contents.value
                else:
                    raise Exception("'%s' is not a valid number of bytes to write.\n", n)
                dxl_addparam_result = DX.DX_XM430_W350.groupBulkWriteAddParam(self.groupwrite_num, id, addr, n, msg, n)
                if dxl_addparam_result != True:
                    print('[ID:%03d ADDR:%d MSG:%d] groupBulkWrite addparam failed\n', id, addr, msgs[i])
                    return
            DX.DX_XM430_W350.groupBulkWriteTxPacket(self.groupwrite_num)
            dxl_comm_result = DX.DX_XM430_W350.getLastTxRxResult(self.port_num, DX.DX_XM430_W350.PROTOCOL_VERSION)
            if dxl_comm_result != DX.DX_XM430_W350.COMM_SUCCESS:
                print('%s\n', DX.DX_XM430_W350.getTxRxResult(DX.DX_XM430_W350.PROTOCOL_VERSION, dxl_comm_result))


