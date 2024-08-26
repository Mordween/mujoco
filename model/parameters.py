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
# robotPosition = [0.18, 0, 0]          # mycobot is smaller than Lite6, so it should be closer
robotRotation = [1, 0, 0, 1]
gripperSize = 0.045

"""
Not yet release : https://mujoco.readthedocs.io/en/latest/changelog.html
"""
autoreset = "disable"   # https://mujoco.readthedocs.io/en/latest/XMLreference.html#option-flag-autoreset


"""
sensor data
"""
store_data = True
csv_filename = 'sensor_data.csv'
# Hard to get the camera data : https://github.com/openai/mujoco-py/issues/249 => didn't work

# Define the directory where the images are stored
image_directory = "cameraPic"  # Use the correct path to your image directory
