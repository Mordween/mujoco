import mujoco
import mujoco.viewer
import time
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3, Twist3
import spatialmath as sm
from spatialmath.base import *
import numpy as np 
import copy


shaftPos = 0.015
timeStep = 1e-3
up_down_speed = 0.0001
previous_time = time.time()
'''
Same as robot_move_to but without drake solver
only with roboticstoolbox-python
'''
def move(viewer, robot, position, quat = [0, 0, -1], numberOfSteps = 500):

        positionR = [   position['y'] - data.body('link_base').xpos[1],
                     - (position['x'] - data.body('link_base').xpos[0]),  
                        position['z'] - data.body('link_base').xpos[2]]
        
        print("position relative : ", positionR)
        robot.q = [     data.joint('joint1').qpos, data.joint('joint2').qpos, 
                        data.joint('joint3').qpos, data.joint('joint4').qpos, 
                        data.joint('joint5').qpos, data.joint('joint6').qpos]
        
        Tep = sm.SE3.Trans(positionR[0], positionR[1], positionR[2]) * sm.SE3.OA([1, 0,1], quat)
        sol = robot.ik_LM(Tep)         # solve IK

        qt = rtb.jtraj(robot.q, sol[0], numberOfSteps)
        previous_time = time.time()

        for steps in range(numberOfSteps):
            qpos = qt.q[steps]
            data.ctrl = [qpos[0], qpos[1], qpos[2], qpos[3], qpos[4], data.ctrl[5], data.ctrl[6], data.ctrl[7], data.ctrl[8], data.ctrl[9], data.ctrl[10], data.ctrl[11]]
            """, qpos[5]"""
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, timeStep-(time.time()-previous_time)))
            previous_time = time.time()

def translateY(viewer, robot, distance, numberOfSteps = 500):
        robot.q = [     data.joint('joint1').qpos, data.joint('joint2').qpos, 
                        data.joint('joint3').qpos, data.joint('joint4').qpos, 
                        data.joint('joint5').qpos, data.joint('joint6').qpos]
        
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
            data.ctrl = [qpos[0], qpos[1], qpos[2], qpos[3], qpos[4], data.ctrl[5], data.ctrl[6], data.ctrl[7], data.ctrl[8], data.ctrl[9], data.ctrl[10], data.ctrl[11]]
            """, qpos[5]"""
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(max(0, timeStep-(time.time()-previous_time)))
            previous_time = time.time()


def crane_move_to(dest, n_sample):
    T_dest = SE3(dest['x'], dest['y'], dest['z'])
    print("t dest", T_dest)
    print("end effector position", model.body('end_effector').pos)
    traj = rtb.ctraj(SE3(model.body('end_effector').pos), T_dest, n_sample)
    previous_time = time.time()
    for i in range(n_sample ):
        # print(SE3.Tx(traj[i].x))
        crane_body_pos = SE3.Tx(traj[i].x)
        end_effector_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)
        beam_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)*SE3.Tz(0.3785) 
        moving_box_pos = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y + shaftPos)*SE3.Tz(0.41)

        #move the differents part of the crane
        model.body('crane_body').pos    = [crane_body_pos.x     , crane_body_pos.y  , crane_body_pos.z]
        model.body('end_effector').pos  = [end_effector_pos.x   , end_effector_pos.y, end_effector_pos.z]
        model.body('beam').pos          = [beam_pos.x           , beam_pos.y        , beam_pos.z]
        model.body('moving_box').pos    = [moving_box_pos.x     , moving_box_pos.y  , moving_box_pos.z]       

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, timeStep-(time.time()-previous_time)))
        previous_time = time.time()

def wait(duration):
    time_pass = time.time() * 1000
    while(time.time()*(1/timeStep) - time_pass < duration*1000):
        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(timeStep)
    previous_time = time.time()




lite6 = rtb.models.Lite6()
lite6.grippers[0].tool = SE3(0, 0, 0.045)
lite6.base = SE3(0.4, 0, 0)*SE3.Rz(pi/2)

xml_path = 'mainV2.xml'

"""
https://mujoco.readthedocs.io/en/latest/XMLreference.html#option
"""
model = mujoco.MjModel.from_xml_path(xml_path)
# model.opt.timestep = 0.002
model.opt.timestep = timeStep
model.opt.iterations = 3
model.opt.solver = 2   # 0 : PGS,  1 : CG, 2 : Newton
print(dir(model.opt))
data = mujoco.MjData(model)

model.body('link_base').pos = [0.4, 0, 0]
model.body('link_base').quat = [1, 0, 0, 1]


T_pick = SE3(2, 3-0.27, 0.3)
# T_place_up = SE3(0.0, 0.2, 0.1)
# T_place = SE3(0, 0.2, 0.09)
T_place_up = SE3(0.0, 2, 2)
T_place = SE3(0, 2, 0.9)

# position = {'x': 0.2, 'y': 0.3, 'z': 0.32}
position = {'x': 0.2, 'y': 0.3, 'z': 0.030 }              # X and Y axes are reversed 
positionShaft = {'x': 0.2, 'y': 0.285, 'z': 0}
positionShaft2 = {'x': 0, 'y': 0.09, 'z': 0} # 0.1 - 0.02/2
lite6Move = 0
shaftUp = 0
craneMove = 0
takeTheBrick = 0

simulation_action = 'init' 


with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    i = 0
    viewer.cam.trackbodyid = 15
    viewer.cam.distance = 1.5
    viewer.cam.lookat = [0, 0, 0]
    viewer.cam.elevation = -45
    viewer.cam.azimuth = 45
    print(dir(model.body('brick')))

    mujoco.mj_step(model, data)
    viewer.sync()

    # print(dir(data.body("gripper_left")))
    # data.body("gripper_left").xipos = [0, 0.1 ,0]
    previous_time = time.time()
    while viewer.is_running():
        
        match simulation_action :
            case 'init' :
                # move(viewer, lite6, position, numberOfSteps=500)
                # robot_move_to(viewer, lite6, position)
                simulation_action = 'rope_init'

            case 'rope_init':
                positionZ = 0.16
                if (data.body('gripper_rope').xpos[2]< positionZ):
                    shaftPos -= up_down_speed
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + shaftPos
                else:
                    simulation_action = 'shaftMove'

            case 'shaftMove' :
                crane_move_to(positionShaft, 1500)
                wait(5)
                simulation_action = 'down_rope'

            case 'down_rope':
                positionZ = 0.046
                if (data.body('gripper_rope').xpos[2]> positionZ):
                    shaftPos += up_down_speed
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + shaftPos
                else:
                    simulation_action = 'take_brick'

            case 'take_brick' :
                data.ctrl = [data.ctrl[0], data.ctrl[1], data.ctrl[2], data.ctrl[3], data.ctrl[4], data.ctrl[5], data.ctrl[6],  data.ctrl[7], data.ctrl[8], 0, 0.05, -0.05]
                wait(5)
                data.ctrl = [data.ctrl[0], data.ctrl[1], data.ctrl[2], data.ctrl[3], data.ctrl[4], data.ctrl[5], data.ctrl[6],  data.ctrl[7], data.ctrl[8], 0, 0.05, -0.05]
                simulation_action = 'up_rope'
            
            case "up_rope":
                positionZ = 0.15
                if (data.body('gripper_rope').xpos[2]< positionZ):
                    shaftPos -= up_down_speed
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + shaftPos
                else:
                    simulation_action = 'shaft_rebase'

            case 'shaft_rebase':
                crane_move_to(positionShaft2, 1500)
                wait(5)
                simulation_action = 'move_robot'
            
            case 'move_robot':
                quat = [0, 1, 0]

                position = {'x': data.body('brick').xpos[0]+0.04, 
                            'y': data.body('brick').xpos[1], 
                            'z': data.body('brick').xpos[2]} 
                    
                print("position of end effector", position)
                move(viewer, lite6, position, quat, numberOfSteps=500)
                wait(5)
                simulation_action = 'turn_end_effector'

            case "turn_end_effector":
                
                # print(data.body('link6').xquat)
                # print("bool : ", (data.body('link6').xquat[1] < 0.5), "value : ", data.body('link6').xquat[1] )

                # if (data.body('link6').xquat[1] > -0.5):
                data.ctrl = [data.ctrl[0], data.ctrl[1], data.ctrl[2], data.ctrl[3], data.ctrl[4], 1.57,
                                    data.ctrl[6], data.ctrl[7], data.ctrl[8], data.ctrl[9], data.ctrl[10], data.ctrl[11]]
                # else:
                simulation_action = 'get_closer'
                wait(5)

            case "get_closer" :
                # quat = [0, 1, 0]
                # position = {'x': data.body('brick').xpos[0], 
                #             'y': data.body('brick').xpos[1], 
                #             'z': data.body('brick').xpos[2]}
                # move(viewer, lite6, position, quat, numberOfSteps=500)
                # translateY(viewer, lite6, -0.04, 500)
                # simulation_action = 'release_brick'
                simulation_action = 'release_brick'
                wait(5)
                                    
            case "release_brick" :
                positionZ = 0.9
                speed = 0.0001
                if (data.body('gripper_rope').xpos[2]> positionZ):
                    shaftPos += speed
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + shaftPos
                else:
                    simulation_action = 'robot_move'
                    data.ctrl = [data.ctrl[0], data.ctrl[1], data.ctrl[2], data.ctrl[3], data.ctrl[4], data.ctrl[5], data.ctrl[6], data.ctrl[7], data.ctrl[8], 0, 0, 0]

            case 'robot_move':
                wait(2)
                # position = {'x': model.body('brick').pos[0], 'y': model.body('brick').pos[1], 'z': model.body('brick').pos[2]} 
                # quat = [model.body('brick').quat[0], model.body('brick').quat[1], model.body('brick').quat[2]]
                # move(viewer, lite6, position, quat, 500)
                simulation_action = 'end'
            
            case 'end' : 
                # print(data.body('moving_box').xpos)
                print("durée de la simulation", round(time.time()-start), " s")
                simulation_action = 'endV2' 

        # print(data.body('link6').xpos)  # position of end effector

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(max(0, timeStep-(time.time()-previous_time)))
        previous_time = time.time()