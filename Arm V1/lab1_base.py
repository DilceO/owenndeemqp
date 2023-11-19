import time
import Robot.py as RB

# Setup robot
travelTime = 2  # Defines the travel time
robot = RB.Robot()  # Creates robot object
robot.writeTime(travelTime)  # Write travel time
robot.writeMotorState(True)  # Write position mode

# Program
robot.writeJoints(0)  # Write joints to zero position
time.sleep(travelTime)  # Wait for trajectory completion
baseWayPoints = [-45, 45, 0]  # Define base waypoints
for baseWayPoint in baseWayPoints:  # Iterate through waypoints
    robot.writeJoints([baseWayPoint, 0, 0, 0])  # Write joint values
    start_time = time.time()  # Start timer
    while time.time() - start_time < travelTime:
        print(robot.getJointsReadings())  # Read joint values

robot.writeGripper(False)
time.sleep(1)

