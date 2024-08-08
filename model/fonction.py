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
        self.data = mujoco.MjData(model)
        self.robot = robot

        self.model.opt.timestep     = param.timeStep
        self.model.opt.iterations   = param.modelIterations
        self.model.opt.solver       = param.modelSolver   # 0 : PGS,  1 : CG, 2 : Newton

        self.model.body('link_base').pos    = param.robotPosition
        self.model.body('link_base').quat   = param.robotRotation


        self.robot.grippers[0].tool = SE3(0, 0, param.gripperSize)
        self.robot.base = SE3(param.robotPosition)*SE3.Rz(pi/2)


    def move(self, viewer, robot, position, quat = [0, 0, -1], numberOfSteps = 100):
        positionR = [   position['y'] - self.data.body('link_base').xpos[1],
                     - (position['x'] - self.data.body('link_base').xpos[0]),  
                        position['z'] - self.data.body('link_base').xpos[2]]
        
        # robot.q = [     self.data.joint('joint1').qpos, self.data.joint('joint2').qpos, 
        #                 self.data.joint('joint3').qpos, self.data.joint('joint4').qpos, 
        #                 self.data.joint('joint5').qpos, self.data.joint('joint6').qpos]
        
        # Tep = sm.SE3.Trans(positionR[0], positionR[1], positionR[2]) * sm.SE3.OA([1, 0,1], quat)
        # sol = robot.ik_LM(Tep)         # solve IK

        # qt = rtb.jtraj(robot.q, sol[0], numberOfSteps)
        Tep = sm.SE3(positionR[0], positionR[1], positionR[2]) * sm.SE3.RPY([quat[0]*90, quat[1]*90, quat[2]*90], order="xyz", unit="deg")
        # Tep = sm.SE3(positionR[0], positionR[1], positionR[2]) * sm.SE3.RPY([0,90,0], order="xyz", unit="deg")
        ctraj = rtb.ctraj(robot.fkine(robot.q), Tep, numberOfSteps)
        jtraj = robot.ikine_LM(ctraj, q0 = robot.q)
        previous_time = time.time()

        for q in jtraj.q:
            qpos = q
            robot.q = q
            # self.data.ctrl = [qpos[0], qpos[1], qpos[2], qpos[3], qpos[4], qpos[5], self.data.ctrl[6], self.data.ctrl[7], self.data.ctrl[8], self.data.ctrl[9], self.data.ctrl[10], self.data.ctrl[11]]
            self.data.joint('joint1').qpos = qpos[0]
            self.data.joint('joint2').qpos = qpos[1]
            self.data.joint('joint3').qpos = qpos[2]
            self.data.joint('joint4').qpos = qpos[3]
            self.data.joint('joint5').qpos = qpos[4]
            self.data.joint('joint6').qpos = qpos[5]
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
()