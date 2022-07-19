import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot

# INITIALIZATION:
# ====================================================
mSim=CoppeliaSim()
ret=mSim.connect(19997)
if ret == -1:
    exit()
else:
    print("Coppelia initialization success...")

# Initialize the robot model
jacoRobot = CoppeliaArmRobot("Jaco")
time.sleep(1)


# MAIN PRORGAM:
# ======================================================