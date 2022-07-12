import time
import numpy as np
import cv2 as cv
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot, CoppeliaSensor

# Initialize the CoppeliaSim connection
mSim = CoppeliaSim()

if mSim.connect(19997) != -1:
    # If the connection is success, initialize the robot properties
    ur10_robot = CoppeliaArmRobot("UR10")
    camera = CoppeliaSensor("/Vision_sensor", 0)
    time.sleep(1)
    res, simImage = camera.getImage()

    # Print camera resolution
    print("Resolution = ", res)

    # Convert image coppeliasim format to opencv format
    img = np.array(simImage, dtype=np.uint8)
    img.resize([res[1], res[0], 3])
    img2 = cv.cvtColor(img, cv.COLOR_RGB2BGR)
    img2 = cv.flip(img2, 0)
    # Show image
    cv.imshow("Vision Sensor", img2)
    cv.waitKey(0)
else:
    print("ERROR: CoppeliaSim connection failed!!!")