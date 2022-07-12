import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot


def pickObject(objName):
    ur10_robot.gripperRelease()

    # Get Object Handle
    objPos = ur10_robot.getObjectPosition(objName)
    print(">>> Pick object ", objName)
    print("Object Position = ", objPos)

    ur10_robot.setSpeed(100, 90)
    pickPos_step1 = [objPos[0] - 100, objPos[1], objPos[2] + 100, 0, 90, 90]
    ur10_robot.setPosition2(pickPos_step1, True)
    ur10_robot.setSpeed(50, 90)
    pickPos_step2 = [objPos[0], objPos[1], objPos[2], 0, 90, 90]
    ur10_robot.setPosition2(pickPos_step2, True)

    time.sleep(0.5)
    ur10_robot.gripperCatch()
    time.sleep(0.5)

    # LiftUp:
    ur10_robot.setSpeed(100, 90)
    lift_pos = [objPos[0], objPos[1], objPos[2] + 300, 0, 90, 90]
    ur10_robot.setPosition2(lift_pos, True)

def putObject(color, step):
    placePos = []
    goalPosLift = []
    # Check Point
    pos = [-110, -15, 105, 0, -90, 90]
    ur10_robot.setJointPosition(pos, True)

    if color == 1:
        placePos = ur10_robot.getObjectPosition("/Plane[2]")
        if step == 1:
            goalPos = [placePos[0] - 50, placePos[1], placePos[2] + 30, 180, 0, 180]
        elif step == 2:
            goalPos = [placePos[0] + 50, placePos[1], placePos[2] + 30, 180, 0, 180]
        elif step == 3:
            goalPos = [placePos[0], placePos[1] - 50, placePos[2] + 80, 180, 0, 90]
        elif step == 4:
            goalPos = [placePos[0], placePos[1] + 50, placePos[2] + 80, 180, 0, 90]

    ur10_robot.setPosition2(goalPos, True)
    time.sleep(0.5)
    ur10_robot.gripperRelease()
    time.sleep(0.5)
    goalPos[2] += 100
    ur10_robot.setPosition2(goalPos, True)
    pos = [0, -15, 105, 90, 90, 90]
    ur10_robot.setJointPosition(pos, True)


# Initialize the CoppeliaSim connection
mSim = CoppeliaSim()
if mSim.connect(19997) != -1:
    # If the connection is success, initialize the robot properties
    ur10_robot = CoppeliaArmRobot("UR10")
    time.sleep(1)

    # Initial Position
    pos = [0, -15, 105, 90, 90, 90]
    #pos = [-80, -15, 105, 0, -90, 90]
    ur10_robot.setJointPosition(pos, True)

    pickObject("/Cuboid[0]")
    putObject(1, 1)

    pickObject("/Cuboid[1]")
    putObject(1, 2)

    pickObject("/Cuboid[2]")
    putObject(1, 3)

    pickObject("/Cuboid[3]")
    putObject(1, 4)

    time.sleep(2)
    mSim.stopSimulation()