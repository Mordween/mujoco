import mujoco
import mujoco.viewer
import time
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
import spatialmath as sm
from spatialmath.base import *

import parameters as param

"""
Libraries used for storing and managing sensor data 
"""
import csv
from PIL import Image
import numpy as np
import os 
import glob


def shaftPosUp(sim):
    param.shaftPos -= param.up_down_speed
    sim.model.body('moving_box').pos[1] = sim.model.body('beam').pos[1] + param.shaftPos

def shaftPosDown(sim):
    param.shaftPos += param.up_down_speed
    sim.model.body('moving_box').pos[1] = sim.model.body('beam').pos[1] + param.shaftPos


class Simulation():
    """
    initialize simulation parameters 
    """
    def __init__(self, model, robot):
        self.model = model
        self.renderer = mujoco.Renderer(model, 480, 640)
        self.data = mujoco.MjData(model)

        self.robot = robot

        self.model.opt.timestep     = param.timeStep
        self.model.opt.iterations   = param.modelIterations
        self.model.opt.solver       = param.modelSolver   # 0 : PGS,  1 : CG, 2 : Newton

        self.model.body('link_base').pos    = param.robotPosition
        self.model.body('link_base').quat   = param.robotRotation

        self.robot.grippers[0].tool = SE3(0, 0, param.gripperSize)
        self.robot.base = SE3(param.robotPosition)*SE3.Rz(pi/2)


        """
        Initialization of the data storage section
        """
        self.iteration = 0

        # Check if the folder exists, if not, create it
        if not os.path.exists(param.image_directory):
            os.makedirs(param.image_directory)

        # Get a list of all .png files in the directory
        images = glob.glob(os.path.join(param.image_directory, "*.png"))

        # Loop through the list and delete each image
        for image in images:
            os.remove(image)
 
        # create .csv file or empty an existing one
        with open(param.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['x', 'y', 'z'])

    """
    this function is used to compute and move the end effector of the robot to a desired position
    """
    def move(self, viewer, robot, position, quat = [0, 0, -1], numberOfSteps = 100):

        # Relative position of the robot's end effector
        positionR = [   position['y'] - self.data.body('link_base').xpos[1],
                     - (position['x'] - self.data.body('link_base').xpos[0]),  
                        position['z'] - self.data.body('link_base').xpos[2]]
        
        Tep = sm.SE3(positionR[0], positionR[1], positionR[2]) * sm.SE3.RPY([quat[0]*90, quat[1]*90, quat[2]*90], order="xyz", unit="deg")
        ctraj = rtb.ctraj(robot.fkine(robot.q), Tep, numberOfSteps)
        jtraj = robot.ikine_LM(ctraj, q0 = robot.q)
        param.previous_time = time.time()

        for q in jtraj.q:
            qpos = q
            robot.q = q
            self.data.ctrl = [qpos[0]           , qpos[1]           , qpos[2]           , qpos[3]           , qpos[4]           , qpos[5], 
                              self.data.ctrl[6] , self.data.ctrl[7] , self.data.ctrl[8] , self.data.ctrl[9] , self.data.ctrl[10], self.data.ctrl[11]]

            self.simStep(viewer)

    """
    this function is used to compute and move the crane from a position to another position
    """
    def crane_move_to(self, viewer, dest, n_sample):
        T_dest = SE3(dest['x'], dest['y'], dest['z'])
        traj = rtb.ctraj(SE3(self.model.body('end_effector').pos), T_dest, n_sample)
        param.previous_time = time.time()
        for i in range(n_sample ):
            crane_body_pos = SE3.Tx(traj[i].x)
            end_effector_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)
            beam_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)*SE3.Tz(0.3785)   # 0.3785 corresponds to shaft position Z
            moving_box_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y + param.shaftPos)*SE3.Tz(0.41)  # 0.41 corresponds to the Z position of the shaft+ a little more than the shaft radius

            # move the differents part of the crane
            self.model.body('crane_body').pos    = [crane_body_pos.x     , crane_body_pos.y  , crane_body_pos.z]
            self.model.body('end_effector').pos  = [end_effector_pos.x   , end_effector_pos.y, end_effector_pos.z]
            self.model.body('beam').pos          = [beam_pos.x           , beam_pos.y        , beam_pos.z]
            self.model.body('moving_box').pos    = [moving_box_pos.x     , moving_box_pos.y  , moving_box_pos.z]       

            self.simStep(viewer)

    """
    this function is used to create a downtime without pausing the simulation as with time.sleep()
    """
    def wait(self, viewer, duration):
        time_pass = time.time()
        param.previous_time = time.time()
        while(time.time() - time_pass < duration):
            self.simStep(viewer)

    """
    this function advances the simulation by one step, updating all dynamics and states and storing data
    """
    def simStep(self, viewer):
        if(param.store_data):
            # Append sensor data to the CSV file (opens the file in append mode)
            with open(param.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                writer.writerow([self.data.sensordata[0], self.data.sensordata[1], self.data.sensordata[2]])
            if (self.iteration % param.captureFrequency == 0):
                self.renderer.update_scene(self.data, camera="robot_cam")
                cam_imgs = []
                cam_img = self.renderer.render()
                cam_imgs.append(cam_img)
                image_arrays = np.array(cam_imgs)

                # reshape from (1, 1, 640, 3) to (640, 3)
                image_array = np.squeeze(image_arrays)
                # Convert the NumPy array to an image
                image = Image.fromarray(image_array)

                # Save the image to a file
                image.save(f'{param.image_directory}/image{self.iteration:05d}.png')
                
        mujoco.mj_step(self.model, self.data)
        viewer.sync()
        time.sleep(max(0, param.timeStep-(time.time()-param.previous_time)))
        param.previous_time = time.time()
        self.iteration +=1
