# =========================================
# Project: CoppeliaSim Connection Test
# June 24, 2022
# @yudarismawahyudi
# =========================================

from include.coppeliasim import CoppeliaSim, Robot

# Initialize Coppelia Lib
mSim = CoppeliaSim()
clientId = mSim.connect(19997)
robot = Robot('UR10')
