import json 

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
# Load the config file
with open('config.json') as config_file:
    config = json.load(config_file)

robot = config["robot"]

# Use the robot name to give the corresponding position and include the right xml file
# Read the XML file
if robot == "lite6":
    robotPosition = [0.4, 0, 0]

    with open('main.xml', 'r+') as file:
        lines = file.readlines()
        lines[4] = f'\t<include file="assets/assets/ufactory_lite6/lite6V2.xml"/>\n'    # include the selected robot
        file.seek(0)  # Move the file pointer back to the beginning of the file
        file.writelines(lines)
        file.truncate()  # Truncate the file to remove any leftover lines

elif robot == "mycobot":
    robotPosition = [0.18, 0, 0]    # mycobot is smaller than Lite6, so it should be closer

    with open('main.xml', 'r+') as file:
        lines = file.readlines()
        lines[4] = f'\t<include file="assets/assets/mycobot_280_pi/mycobot.xml"/>\n'
        file.seek(0)  # Move the file pointer back to the beginning of the file
        file.writelines(lines)
        file.truncate()  # Truncate the file to remove any leftover lines


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

# File where IMU data are stored
csv_filename = 'sensor_data.csv'

# Path to robot image directory
image_directory = "cameraPic" 

# Number of iterations between each image capture
captureFrequency = 100          
