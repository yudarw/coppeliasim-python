import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot

# Initialize the CoppeliaSim connection
mSim = CoppeliaSim()
if mSim.connect(19997) != -1:
    # If the connection is success, initialize the robot properties
    ur10_robot = CoppeliaArmRobot("UR10")
    time.sleep(0.5)
    # Then try to read the position of the robot
    currentPos = ur10_robot.readPosition()
    print("Current Robot Position = ", currentPos)

    # Example of set robot position
    pos1 = [400, -300, 100, 180, 0, 0]
    pos2 = [400, -300, 200, 180, 0, 0]
    pos3 = [400, 300, 200, 180, 0, 0]
    pos4 = [400, 300, 100, 180, 0, 0]

    ur10_robot.gripperRelease()
    while True:
        ur10_robot.setSpeed(2000, 50)
        ur10_robot.setPosition2(pos2, True)
        ur10_robot.setPosition2(pos1, True)
        time.sleep(1)
        ur10_robot.gripperCatch()
        time.sleep(1)
        ur10_robot.setPosition2(pos2, True)

        ur10_robot.setSpeed(50, 50)
        ur10_robot.setPosition2(pos3, True)
        ur10_robot.setPosition2(pos4, True)
        time.sleep(1)
        ur10_robot.gripperRelease()
        time.sleep(1)
        ur10_robot.setPosition2(pos3, True)

    print('Program Finished')

else:
    pass

