from include.coppeliasim import CoppeliaSim, Robot
import sim

coppelia = CoppeliaSim()
coppelia.connect(19997)

clientId = coppelia.clientId
robot = Robot('UR5')
pos = robot.read_position()
print('Initial Robot Position: ', pos)
robot.set_speed(50)
pos1 = [200, 300, 250, 180, 0, 0]
pos2 = [400, 340, 250, 180, 10, 0]
pos3 = [400, -300, 250, 180, 15, 0]
pos4 = [200, -300, 250, 180, -15, 0]

ret, handle = sim.simxGetObjectHandle(coppelia.clientId, 'Proximity_sensor', sim.simx_opmode_oneshot_wait)

while True:
    state = sim.simxReadProximitySensor(clientId, handle, sim.simx_opmode_oneshot_wait)
    print('Proximity sensor: ', state)


''''
robot.set_position2(pos1, True)
robot.gripper(True)
time.sleep(1);
robot.set_position2(pos2, True)
robot.set_position2(pos3, True)
robot.gripper(False)
time.sleep(0.5)
robot.set_position2(pos4, True)
time.sleep(0.5)
sim.disconnect()
'''


