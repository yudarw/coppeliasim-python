# =========================================
# Project: CoppeliaSim Pick and Place
# June 24, 2022
# @yudarismawahyudi
# =========================================

from coppeliasim import CoppeliaSim, Robot
import time
import sim

# Initialize Coppelia Lib
mSim = CoppeliaSim()
clientId = mSim.connect(19997)
robot = Robot('UR5')

# Try Connection:
time.sleep(1)

# Define function
def pick_object(pos):
    liftUp_pos = pos.copy()
    liftUp_pos[2] += 100

    robot.set_speed(100)
    robot.set_position2(pos, True)
    robot.gripper(0)
    time.sleep(0.1)
    robot.set_position2(liftUp_pos, True)

def put_object(pos):
    liftUp_pos = pos.copy()
    liftUp_pos[2] += 100

    robot.set_speed(100)
    robot.set_position2(liftUp_pos, True)
    robot.set_position2(pos, True)
    robot.gripper(1)
    robot.set_position2(liftUp_pos, True)




if clientId != -1:
    # Retrieve object handle:
    pickPos = robot.get_object_position('/pick_pos')
    goalPos = robot.get_object_position('/goal_pos')

    # Initialize proximity sensor
    ret, proxHandle = sim.simxGetObjectHandle(clientId, '/Proximity_sensor', sim.simx_opmode_oneshot_wait)
    sim.simxReadProximitySensor(clientId, proxHandle, sim.simx_opmode_streaming)

    initPos = pickPos.copy()
    initPos[2] = 100

    print(pickPos)
    print(initPos)

    robot.set_position2(initPos, True)
    robot.gripper(1)
    time.sleep(1)

    while True:
        # Read proximity sensor
        ret = sim.simxReadProximitySensor(clientId, proxHandle, sim.simx_opmode_buffer)
        proxState = ret[1]

        if proxState:
            pick_object(pickPos)
            put_object(goalPos)
            robot.set_position2(initPos, True)

        time.sleep(0.02)



else :
    pass