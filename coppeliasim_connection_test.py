# =========================================
# Project: CoppeliaSim Connection Test
# June 24, 2022
# @yudarismawahyudi
# =========================================

from coppeliasim import CoppeliaSim, Robot
import time
import sim

# Initialize Coppelia Lib
mSim = CoppeliaSim()
clientId = mSim.connect(19997)
robot = Robot('UR10')
