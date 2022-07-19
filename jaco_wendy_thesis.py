import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot
import threading

# Position Data:
pickPos = [400, -150, 0, 180, 0, 0]
liftPos1 = [400, -150, 400, 180, 0, 0]
goalPos = [500, 200, 0, 180, 0, 0]
liftPos2 = [500, 200, 500, 180, 60, 0]
liftPos3 = [500, 200, 100, 180, 0, 0]


# THREAD FUNCTION:
# Multithread function khusus buat menggerakkan robot
# Program ini akan terus melalukan looping pick & place
# dan tidak terpengaruh dengan main program (pose detection)
def thread_robotMovement():
    while True:
        jacoRobot.gripperRelease()
        jacoRobot.setPosition2(liftPos1, True)
        jacoRobot.setPosition2(pickPos, True)
        time.sleep(1)
        jacoRobot.gripperCatch()
        jacoRobot.setPosition2(liftPos1, True)
        jacoRobot.setPosition2(liftPos2, True)
        jacoRobot.setPosition2(goalPos, True)
        jacoRobot.gripperRelease()
        time.sleep(1)
        jacoRobot.setPosition2(liftPos3, True)


# INITIALIZATION:
# ====================================================
mSim=CoppeliaSim()
ret=mSim.connect(19997)
if ret == -1:
    exit()

# Initialize the robot model
jacoRobot = CoppeliaArmRobot("Jaco")
time.sleep(1)

# start thread:
t = threading.Thread(target=thread_robotMovement)
t.start()

# MAIN PRORGAM:
# ======================================================

# masukkan program utama disini (looping program)
while True:
    pass