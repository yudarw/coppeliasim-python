# =========================================
# CoppeliaSim Pick and Place
#
# June 24, 2022
# @yudarismawahyudi
# =========================================


from include.coppeliasim import CoppeliaSim, Robot
import time
import sim
import cv2 as cv
import numpy as np


# ============================================================= #
#  Color Image Filtering :
# ============================================================= #
def filter_image(img, filterColor):

    if filterColor == 'red':
        lower_bound = np.array([0, 70, 50])
        upper_bound = np.array([10, 255, 255])
    elif filterColor == 'green':
        lower_bound = np.array([36, 25, 25])
        upper_bound = np.array([70, 255, 255])
    elif filterColor == 'blue':
        lower_bound = np.array([105, 70, 50])
        upper_bound = np.array([130, 255, 255])

    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    # Create a mask using the bounds set
    mask = cv.inRange(hsv, lower_bound, upper_bound)
    # Create an inverse of the mask
    mask_inv = cv.bitwise_not(mask)
    # Filter only the red colour from the original image using the mask (foreground)
    res = cv.bitwise_and(img, img, mask=mask)
    # Filter the regions containing colours other than red from the grayscale image
    background = cv.bitwise_and(gray, gray, mask=mask_inv)
    # Convert the one channelled grayscale background to a three channelled image
    background = np.stack((background,) * 3, axis=-1)
    # add the foreground and the background
    added_img = cv.add(res, background)

    return mask

# ============================================================= #
#  Orientation Detection :
# ============================================================= #
def detect_orientation(img):
    # Convert image to grayscale:
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Convert image to binary:
    ret, bw = cv.threshold(gray, 50, 255, cv.THRESH_BINARY | cv.THRESH_OTSU)

    red = filter_image(img, 'red')
    green = filter_image(img, 'green')
    blue = filter_image(img, 'blue')

    combine = cv.bitwise_and(red, red, mask=green)

    contours, res = cv.findContours(green, cv.RETR_LIST, cv.CHAIN_APPROX_NONE)

    for i, c in enumerate(contours):

        # Calculate the area of each contour
        area = cv.contourArea(c)

        # Ignore contours that are too small or too large
        if area < 3700 or 100000 < area:
            continue

        # cv.minAreaRect returns:
        # (center(x, y), (width, height), angle of rotation) = cv2.minAreaRect(c)
        rect = cv.minAreaRect(c)
        box = cv.boxPoints(rect)
        box = np.int0(box)

        # Retrieve the key parameters of the rotated bounding box
        center = (int(rect[0][0]), int(rect[0][1]))
        width = int(rect[1][0])
        height = int(rect[1][1])
        angle = int(rect[2])

        if width < height:
            angle = 90 - angle
        else:
            angle = -angle

        #label = "(" + str(angle) + " deg"

        label = "({x}, {y}, Angle={ori})".format(x=center[0], y=center[1], ori=angle)

        #textbox = cv.rectangle(img, (center[0] - 35, center[1] - 25),
        #                       (center[0] + 295, center[1] + 10), (255, 255, 255), -1)

        cv.putText(img, label, (center[0], center[1]),
                   cv.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv.LINE_AA)

        cv.drawContours(img, [box], 0, (0, 0, 255), 2)

    cv.imshow('Output Image', img)

    return img





# Initialize Coppelia Library
mSim = CoppeliaSim()
mSim.connect(19997)
robot = Robot('UR5')

# Retrieve object position
pick_pos = robot.get_object_position('/pick_pos')

# Command robot to move to picking position
robot.gripper(1)
robot.set_position(pick_pos)

# Retrieve vision sensor
ret, vHandle = sim.simxGetObjectHandle(mSim.clientId, '/Vision_sensor', sim.simx_opmode_oneshot_wait)

# Get the image
ret, resolution, image = sim.simxGetVisionSensorImage(mSim.clientId, vHandle, 0, sim.simx_opmode_streaming)

time.sleep(1)


while sim.simxGetConnectionId(mSim.clientId) != -1:
    ret, resolution, image = sim.simxGetVisionSensorImage(mSim.clientId, vHandle, 0, sim.simx_opmode_buffer)
    if ret == sim.simx_return_ok :
        # print("Image OK!!!")
        img = np.array(image, dtype=np.uint8)
        img.resize([resolution[1], resolution[0], 3])
        img2 = cv.cvtColor(img, cv.COLOR_RGB2BGR)
        img2 = cv.flip(img2, 0)
        #cv.imshow('image', img2)
        #imgFilter = filter_image(img2, 'red')
        detect_orientation(img2)

    elif ret == sim.simx_return_novalue_flag:
        print('No image yet')
        pass

    cv.waitKey(30)
