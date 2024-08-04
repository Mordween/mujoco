xml_path = 'mainV2.xml'


"""
Simulation parameters
"""
shaftPos = 0.015
timeStep = 1e-3
up_down_speed = 0.0002


"""
Simulator parameters
https://mujoco.readthedocs.io/en/latest/XMLreference.html#option
"""
modelTimestep = timeStep
modelIterations = 3
modelSolver = 2         # 0 : PGS,  1 : CG, 2 : Newton

previous_time = 0