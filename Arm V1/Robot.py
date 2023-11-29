# (c) 2023 Robotics Engineering Department, WPI
# Skeleton Robot class for OpenManipulator-X Robot for RBE 3001
import OM_X_arm as OM
import DX_XM430_W350 as DX
import numpy as np

class Robot(OM.OM_X_arm):
    # Many properties are abstracted into OM_X_arm and DX_XM430_W350. classes
    # Hopefully, you should only need what's in this class to accomplish everything.
    # But feel free to poke around!
    def __init__(self):
        # Super class constructor called implicitly
        # Add startup functionality here
        super().__init__()
        # Change robot to position mode with torque enabled by default
        # Feel free to change this as desired
        self.writeMode('p')
        self.writeMotorState(True)
        # Set the robot to move between positions with a 5 second profile
        # change here or call writeTime in scripts to change
        self.writeTime(2)
        # Robot Dimensions
        self.mDim = [96.326, 130.23, 124, 133.4] # (mm)
        self.mOtherDim = [128, 24] # (mm)
        self.mDHTable = [[0, self.mDim[0], 0, -90],
                         [(np.asind(self.mOtherDim[1]/self.mOtherDim[0])) - 90, 0, self.mDim[1], 0],
                         [(90 - np.asind(self.mOtherDim[1]/self.mOtherDim[0])), 0, self.mDim[2], 0],
                         [0, 0, self.mDim[3], 0]]
    
    # Sends the joints to the desired angles
    # goals [1x4 double] - angles (degrees) for each of the joints to go to
    def writeJoints(self, goals):
        goals = np.mod(np.round(goals * DX.DX_XM430_W350.TICKS_PER_DEG + DX.DX_XM430_W350.TICK_POS_OFFSET), DX.DX_XM430_W350.TICKS_PER_ROT)
        self.bulkReadWrite(DX.DX_XM430_W350.POS_LEN, DX.DX_XM430_W350.GOAL_POSITION, goals)
    
    # Creates a time based profile (trapezoidal) based on the desired times
    # This will cause writePosition to take the desired number of
    # seconds to reach the setpoint. Set time to 0 to disable this profile (be careful).
    # time [double] - total profile time in s. If 0, the profile will be disabled (be extra careful).
    # acc_time [double] - the total acceleration time (for ramp up and ramp down individually, not combined)
    # acc_time is an optional parameter. It defaults to time/3.
    def writeTime(self, time, acc_time=None):
        if acc_time is None:
            acc_time = time / 3
        time_ms = time * DX.DX_XM430_W350.MS_PER_S
        acc_time_ms = acc_time * DX.DX_XM430_W350.MS_PER_S
        print("time")
        print(time_ms)
        print("acc time")
        print(acc_time_ms)
        self.bulkReadWrite(DX.DX_XM430_W350.PROF_ACC_LEN, DX.DX_XM430_W350.PROF_ACC, acc_time_ms)
        self.bulkReadWrite(DX.DX_XM430_W350.PROF_VEL_LEN, DX.DX_XM430_W350.PROF_VEL, time_ms)
    
    # Sets the gripper to be open or closed
    # Feel free to change values for open and closed positions as desired (they are in degrees)
    # open [boolean] - true to set the gripper to open, false to close
    def writeGripper(self, open):
        if open:
            self.gripper.writePosition(-35)
        else:
            self.gripper.writePosition(55)
    
    # Sets position holding for the joints on or off
    # enable [boolean] - true to enable torque to hold last set position for all joints, false to disable
    def writeMotorState(self, enable):
        self.bulkReadWrite(DX.DX_XM430_W350.TORQUE_ENABLE_LEN, DX.DX_XM430_W350.TORQUE_ENABLE, enable)
    
    # Supplies the joints with the desired currents
    # currents [1x4 double] - currents (mA) for each of the joints to be supplied
    def writeCurrents(self, currents):
        currentInTicks = np.round(currents * DX.DX_XM430_W350.TICKS_PER_mA)
        self.bulkReadWrite(DX.DX_XM430_W350.CURR_LEN, DX.DX_XM430_W350.GOAL_CURRENT, currentInTicks)
    
    # Change the operating mode for all joints:
    # https://emanual.robotis.com/docs/en/dxl/x/xm430-w350/#operating-mode11
    # mode [string] - new operating mode for all joints
    # "current": Current Control Mode (writeCurrent)
    # "velocity": Velocity Control Mode (writeVelocity)
    # "position": Position Control Mode (writePosition)
    # Other provided but not relevant/useful modes:
    # "ext position": Extended Position Control Mode
    # "curr position": Current-based Position Control Mode
    # "pwm voltage": PWM Control Mode
    def writeMode(self, mode):
        if mode in ['current', 'c']:
            writeMode = DX.DX_XM430_W350.CURR_CNTR_MD
        elif mode in ['velocity', 'v']:
            writeMode = DX.DX_XM430_W350.VEL_CNTR_MD
        elif mode in ['position', 'p']:
            writeMode = DX.DX_XM430_W350.POS_CNTR_MD
        elif mode in ['ext position', 'ep']: # Not useful normally
            writeMode = DX.DX_XM430_W350.EXT_POS_CNTR_MD
        elif mode in ['curr position', 'cp']: # Not useful normally
            writeMode = DX.DX_XM430_W350.CURR_POS_CNTR_MD
        elif mode in ['pwm voltage', 'pwm']: # Not useful normally
            writeMode = DX.DX_XM430_W350.PWM_CNTR_MD
        else:
            raise ValueError("setOperatingMode input cannot be '{}'. See implementation in DX_XM430_W350. class.".format(mode))
        lastVelTimes = self.bulkReadWrite(DX.DX_XM430_W350.PROF_VEL_LEN, DX.DX_XM430_W350.PROF_VEL)
        lastAccTimes = self.bulkReadWrite(DX.DX_XM430_W350.PROF_ACC_LEN, DX.DX_XM430_W350.PROF_ACC)
        self.writeMotorState(False)
        self.bulkReadWrite(DX.DX_XM430_W350.OPR_MODE_LEN, DX.DX_XM430_W350.OPR_MODE, writeMode)
        self.writeTime(lastVelTimes[0] / 1000, lastAccTimes[0] / 1000)
        self.writeMotorState(True)
    
    # Gets the current joint positions, velocities, and currents
    # readings [3x4 double] - The joints' positions, velocities,
    # and efforts (deg, deg/s, mA)
    def getJointsReadings(self):
        readings = np.zeros((3,4))
        readings[0, :] = (self.bulkReadWrite(DX.DX_XM430_W350.POS_LEN, DX.DX_XM430_W350.CURR_POSITION) - DX.DX_XM430_W350.TICK_POS_OFFSET) / DX.DX_XM430_W350.TICKS_PER_DEG
        readings[1, :] = self.bulkReadWrite(DX.DX_XM430_W350.VEL_LEN, DX.DX_XM430_W350.CURR_VELOCITY) / DX.DX_XM430_W350.TICKS_PER_ANGVEL
        readings[2, :] = self.bulkReadWrite(DX.DX_XM430_W350.CURR_LEN, DX.DX_XM430_W350.CURR_CURRENT) / DX.DX_XM430_W350.TICKS_PER_mA
        return readings
    
    # Sends the joints at the desired velocites
    # vels [1x4 double] - angular velocites (deg/s) for each of the joints to go at
    def writeVelocities(self, vels):
        vels = np.round(vels * DX.DX_XM430_W350.TICKS_PER_ANGVEL)
        self.bulkReadWrite(DX.DX_XM430_W350.VEL_LEN, DX.DX_XM430_W350.GOAL_VELOCITY, vels)


