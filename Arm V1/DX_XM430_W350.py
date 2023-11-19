import ctypes

class DX_XM430_W350:
    # Constants
    LIB_NAME = 'libdxl_x64_c'  # Library for Linux, change if other OS
    BAUDRATE = 1000000
    PROTOCOL_VERSION = 2.0
    COMM_SUCCESS = 0
    COMM_TX_FAIL = -1001

    # Control Table Constants: https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#control-table
    DRIVE_MODE = 10
    OPR_MODE = 11
    TORQUE_ENABLE = 64
    LED = 65
    GOAL_CURRENT = 102
    GOAL_VELOCITY = 104
    PROF_ACC = 108
    PROF_VEL = 112
    GOAL_POSITION = 116
    CURR_CURRENT = 126
    CURR_VELOCITY = 128
    CURR_POSITION = 132

    # message lengths in bytes
    DRIVE_MODE_LEN = 1
    VEL_PROF = 0
    TIME_PROF = 4
    OPR_MODE_LEN = 1
    CURR_CNTR_MD = 0
    VEL_CNTR_MD = 1
    POS_CNTR_MD = 3
    EXT_POS_CNTR_MD = 4
    CURR_POS_CNTR_MD = 5
    PWM_CNTR_MD = 16
    TORQUE_ENABLE_LEN = 1
    LED_LEN = 1
    PROF_ACC_LEN = 4
    PROF_VEL_LEN = 4
    CURR_LEN = 2
    VEL_LEN = 4
    POS_LEN = 4

    # Unit Conversions
    MS_PER_S = 1000
    TICKS_PER_ROT = 4096
    TICK_POS_OFFSET = TICKS_PER_ROT / 2  # position value for a joint angle of 0 (2048 for this case)
    TICKS_PER_DEG = TICKS_PER_ROT / 360
    TICKS_PER_ANGVEL = 1 / (0.229 * 6)  # 1 tick = 0.229 rev/min = 0.229*360/60 deg/s
    TICKS_PER_mA = 1 / 2.69  # 1 tick = 2.69 mA

    def __init__(self, id, device_name):
        self.id = id
        self.device_name = device_name

        # Load Libraries
        if not libisloaded(self.LIB_NAME):
            notfound, warnings = loadlibrary(self.LIB_NAME, 'dynamixel_sdk.h', 'addheader', 'port_handler.h',
                                             'addheader', 'packet_handler.h', 'addheader', 'group_bulk_read.h',
                                             'addheader', 'group_bulk_write.h')
        self.port_num = portHandler(self.device_name)

        self.startConnection()

    def startConnection(self):
        packetHandler()

        # Open port
        if not openPort(self.port_num):
            unloadlibrary(self.LIB_NAME)
            print('Failed to open the port!\n')
            input('Press any key to terminate...\n')
            return

        # Set port baudrate
        if not setBaudRate(self.port_num, self.BAUDRATE):
            unloadlibrary(self.LIB_NAME)
            print('Failed to change the baudrate!\n')
            input('Press any key to terminate...\n')
            return

    def stopConnection(self):
        closePort(self.port_num)

    def getJointReadings(self):
        readings = [0, 0, 0]

        cur_pos = double(self.readWriteByte(self.POS_LEN, self.CURR_POSITION))
        cur_vel = double(self.readWriteByte(self.VEL_LEN, self.CURR_VELOCITY))
        cur_curr = double(self.readWriteByte(self.CURR_LEN, self.CURR_CURRENT))

        cur_pos = (cur_pos - self.TICK_POS_OFFSET) / self.TICKS_PER_DEG
        cur_vel = cur_vel / self.TICKS_PER_ANGVEL
        cur_curr = cur_curr / self.TICKS_PER_mA

        readings[0] = cur_pos
        readings[1] = cur_vel
        readings[2] = cur_curr

        return readings

    def writePosition(self, angle):
        position = mod(round(angle * self.TICKS_PER_DEG + self.TICK_POS_OFFSET), self.TICKS_PER_ROT)
        self.readWriteByte(self.POS_LEN, self.GOAL_POSITION, position)

    def writeTime(self, time, acc_time=None):
        if acc_time is None:
            acc_time = time / 3

        time_ms = time * self.MS_PER_S
        acc_time_ms = acc_time * self.MS_PER_S

        self.readWriteByte(self.PROF_ACC_LEN, self.PROF_ACC, acc_time_ms)
        self.readWriteByte(self.PROF_VEL_LEN, self.PROF_VEL, time_ms)

    def writeVelocity(self, velocity):
        velTicks = velocity * self.TICKS_PER_ANGVEL
        self.readWriteByte(self.VEL_LEN, self.GOAL_VELOCITY, velTicks)

    def writeCurrent(self, current):
        currentInTicks = current * self.TICKS_PER_mA
        self.readWriteByte(self.CURR_LEN, self.GOAL_CURRENT, currentInTicks)

    def toggleTorque(self, enable):
        self.readWriteByte(self.TORQUE_ENABLE_LEN, self.TORQUE_ENABLE, enable)

    def toggleLED(self, enable):
        self.readWriteByte(self.LED_LEN, self.LED, enable)

    def setOperatingMode(self, mode):
        currentMode = self.readWriteByte(self.TORQUE_ENABLE_LEN, self.TORQUE_ENABLE)

        self.toggleTorque(False)

        if mode in ['current', 'c']:
            writeMode = self.CURR_CNTR_MD
        elif mode in ['velocity', 'v']:
            writeMode = self.VEL_CNTR_MD
        elif mode in ['position', 'p']:
            writeMode = self.POS_CNTR_MD
        elif mode in ['ext position', 'ep']:
            writeMode = self.EXT_POS_CNTR_MD
        elif mode in ['curr position', 'cp']:
            writeMode = self.CURR_POS_CNTR_MD
        elif mode in ['pwm voltage', 'pwm']:
            writeMode = self.PWM_CNTR_MD
        else:
            raise ValueError(f"setOperatingMode input cannot be '{mode}'. See implementation in DX_XM430_W350 class.")

        self.readWriteByte(self.OPR_MODE_LEN, self.OPR_MODE, writeMode)
        self.toggleTorque(currentMode)

    def checkPacket(self, addr, msg=None):
        dxl_comm_result = getLastTxRxResult(self.port_num, self.PROTOCOL_VERSION)
        dxl_error = getLastRxPacketError(self.port_num, self.PROTOCOL_VERSION)
        packetError = (dxl_comm_result != self.COMM_SUCCESS) or (dxl_error != 0)

        if msg is not None and packetError:
            print(f'[msg] {msg}\n')

        if dxl_comm_result != self.COMM_SUCCESS:
            print(f'[addr:{addr}] {getTxRxResult(self.PROTOCOL_VERSION, dxl_comm_result)}\n')
            raise Exception("Communication Error: See above.")
        elif dxl_error != 0:
            print(f'[addr:{addr}] {getRxPacketError(self.PROTOCOL_VERSION, dxl_error)}\n')
            raise Exception("Received Error Packet: See above.")

    def readWriteByte(self, n, addr, msg=None):
        if msg is not None:
            msg = round(msg)
            if n == 1:
                msg = ctypes.c_uint8(msg).value
                write1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr, msg)
            elif n == 2:
                msg = ctypes.c_uint16(msg).value
                write2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr, msg)
            elif n == 4:
                msg = ctypes.c_uint32(msg).value
                write4ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr, msg)
            else:
                raise ValueError(f"'{n}' is not a valid number of bytes to write.\n")

            self.checkPacket(addr, msg)
        else:
            if n == 1:
                result = read1ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr)
                result = ctypes.c_int8(result).value
            elif n == 2:
                result = read2ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr)
                result = ctypes.c_int16(result).value
            elif n == 4:
                result = read4ByteTxRx(self.port_num, self.PROTOCOL_VERSION, self.id, addr)
                result = ctypes.c_int32(result).value
            else:
                raise ValueError(f"'{n}' is not a valid number of bytes to read.\n")

            self.checkPacket(addr)

        return result
