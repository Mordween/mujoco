xml_path = 'main.xml'


"""
Simulation parameters
"""
shaftPos = 0.015
timeStep = 0.002  # 1e-3
up_down_speed = 0.0002
previous_time = 0

"""
Simulator parameters
https://mujoco.readthedocs.io/en/latest/XMLreference.html#option
"""
modelTimestep = timeStep
modelIterations = 1000
modelSolver = 1         # 0 : PGS,  1 : CG, 2 : Newton


"""
Robot parameter
"""
robotPosition = [0.4, 0, 0]
robotRotation = [1, 0, 0, 1]
gripperSize = 0.045

"""
Not yet release : https://mujoco.readthedocs.io/en/latest/changelog.html
"""
autoreset = "disable"   # https://mujoco.readthedocs.io/en/latest/XMLreference.html#option-flag-autoreset
