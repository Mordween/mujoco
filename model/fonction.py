import mujoco
import mujoco.viewer
import time
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3
import spatialmath as sm
from spatialmath.base import *

import parameters as param


def shaftPosUp():
    param.shaftPos -= param.up_down_speed

def shaftPosDown():
    param.shaftPos += param.up_down_speed

class Simulation():
    def __init__(self, model, robot):
        self.model = model
        self.data = data = mujoco.MjData(model)
        self.robot = robot

        self.model.opt.timestep     = param.timeStep
        self.model.opt.iterations   = param.modelIterations
        self.model.opt.solver       = param.modelSolver   # 0 : PGS,  1 : CG, 2 : Newton

        self.model.body('link_base').pos = [0.4, 0, 0]
        self.model.body('link_base').quat = [1, 0, 0, 1]

        self.robot.grippers[0].tool = SE3(0, 0, 0.045)
        self.robot.base = SE3(0.4, 0, 0)*SE3.Rz(pi/2)


    def move(self, viewer, robot, position, quat = [0, 0, -1], numberOfSteps = 500):
        positionR = [   position['y'] - self.data.body('link_base').xpos[1],
                    - (position['x'] - self.data.body('link_base').xpos[0]),  
                        position['z'] - self.data.body('link_base').xpos[2]]
        
        robot.q = [     self.data.joint('joint1').qpos, self.data.joint('joint2').qpos, 
                        self.data.joint('joint3').qpos, self.data.joint('joint4').qpos, 
                        self.data.joint('joint5').qpos, self.data.joint('joint6').qpos]
        
        Tep = sm.SE3.Trans(positionR[0], positionR[1], positionR[2]) * sm.SE3.OA([1, 0,1], quat)
        sol = robot.ik_LM(Tep)         # solve IK

        qt = rtb.jtraj(robot.q, sol[0], numberOfSteps)
        previous_time = time.time()
        for steps in range(numberOfSteps):

            qpos = qt.q[steps]
            self.data.ctrl = [qpos[0], qpos[1], qpos[2], qpos[3], qpos[4], self.data.ctrl[5], self.data.ctrl[6], self.data.ctrl[7], self.data.ctrl[8], self.data.ctrl[9], self.data.ctrl[10], self.data.ctrl[11]]

            mujoco.mj_step(self.model, self.data)
            viewer.sync()
            time.sleep(max(0, param.timeStep-(time.time()-previous_time)))
            previous_time = time.time()


    """
    Not use 
    """
    def translateY(self, viewer, robot, distance, numberOfSteps = 500):
            robot.q = [     self.data.joint('joint1').qpos, self.data.joint('joint2').qpos, 
                            self.data.joint('joint3').qpos, self.data.joint('joint4').qpos, 
                            self.data.joint('joint5').qpos, self.data.joint('joint6').qpos]
            
            Tep = sm.SE3.Ty(distance)* sm.SE3.OA([1, 0,1], [0, 1, 0])           # https://bdaiinstitute.github.io/spatialmath-python/3d_pose_SE3.html
            sol = robot.ik_LM(Tep)         # solve IK

            # qt = rtb.jtraj(robot.q, sol[0], numberOfSteps
            qt = rtb.jtraj(robot.q, distance, numberOfSteps)
            # print(data.body('link6').xquat)
            # end_effector = data.body('link6').xpos
            # end_effectorP = SE3(end_effector)
            # qt = rtb.jtraj()
            previous_time = time.time()
            for steps in range(numberOfSteps):

                qpos = qt.q[steps]
                self.data.ctrl = [qpos[0], qpos[1], qpos[2], qpos[3], qpos[4], self.data.ctrl[5], self.data.ctrl[6], self.data.ctrl[7], self.data.ctrl[8], self.data.ctrl[9], self.data.ctrl[10], self.data.ctrl[11]]
                """, qpos[5]"""
                mujoco.mj_step(self.model, self.data)
                viewer.sync()
                time.sleep(max(0, param.timeStep-(time.time()-previous_time)))
                previous_time = time.time()


    def crane_move_to(self, viewer, dest, n_sample):
        T_dest = SE3(dest['x'], dest['y'], dest['z'])
        traj = rtb.ctraj(SE3(self.model.body('end_effector').pos), T_dest, n_sample)
        previous_time = time.time()
        for i in range(n_sample ):
            crane_body_pos = SE3.Tx(traj[i].x)
            end_effector_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)
            beam_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)*SE3.Tz(0.3785) 
            moving_box_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y + param.shaftPos)*SE3.Tz(0.41)

            #move the differents part of the crane
            self.model.body('crane_body').pos    = [crane_body_pos.x     , crane_body_pos.y  , crane_body_pos.z]
            self.model.body('end_effector').pos  = [end_effector_pos.x   , end_effector_pos.y, end_effector_pos.z]
            self.model.body('beam').pos          = [beam_pos.x           , beam_pos.y        , beam_pos.z]
            self.model.body('moving_box').pos    = [moving_box_pos.x     , moving_box_pos.y  , moving_box_pos.z]       

            mujoco.mj_step(self.model, self.data)
            viewer.sync()
            time.sleep(max(0, param.timeStep-(time.time()-previous_time)))
            previous_time = time.time()

    def wait(self, viewer, duration):
        time_pass = time.time() * (1/param.timeStep)
        previous_time = time.time()
        while(time.time()*(1/param.timeStep) - time_pass < duration*(1/param.timeStep)):
            mujoco.mj_step(self.model, self.data)
            viewer.sync()
            time.sleep(max(0, param.timeStep-(time.time()-previous_time)))
            previous_time = time.time()
