import mujoco
import mujoco.viewer
import time
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3, Twist3
import spatialmath as sm
from spatialmath.base import *
# from pydrake.solvers import MathematicalProgram, Solve
from pydrake.all import( MathematicalProgram, Solve)
import numpy as np 
import copy


shaftPos = 0.015

def jacobian_i_k_optimisation(robot, v, qd_max=1):

    # jacobian inverse kinematics with optimisation
    J = robot.jacobe(robot.q)
    prog = MathematicalProgram()
    qd_opt = prog.NewContinuousVariables(6, "v_opt")
    # Define the error term for the cost function
    error = J @ qd_opt - v
    prog.AddCost(error.dot(error))
    # Add bounding box constraint for joint velocities
    lower_bounds = [-qd_max] * 6  # Lower bounds for each joint velocity
    upper_bounds = [qd_max] * 6   # Upper bounds for each joint velocity
    prog.AddBoundingBoxConstraint(lower_bounds, upper_bounds, qd_opt)
    # Solve the optimization problem
    result = Solve(prog)
    return result.is_success(), result.GetSolution(qd_opt)


def robot_move_to(viewer, robot, position, gain=2, treshold=0.001, qd_max=1): 
        arrived = False

        # desired relative position of the end effector
        dest = SE3(position['x'] - data.body('link_base').xpos[0], 
                   position['y'] - data.body('link_base').xpos[1], 
                   position['z'] - data.body('link_base').xpos[2])

        # position of the end effector one step ahead
        pos_nm1 = (data.body('link6').xpos).copy()

        while not arrived:
            # update robot joint position
            robot.q = [ data.joint('joint1').qpos, data.joint('joint2').qpos, 
                        data.joint('joint3').qpos, data.joint('joint4').qpos, 
                        data.joint('joint5').qpos, data.joint('joint6').qpos]
            
            # difference between position of the end effector and position of the base == relative position of the end effector
            end_effector = SE3(data.body('link6').xpos - data.body('link_base').xpos)
            #robot.fkine(robot.q)       # relative position of the end effector
            
            if isinstance(dest, SE3) or (isinstance(dest, np.ndarray) and dest.shape==(4,4)):
                v, arrived = rtb.p_servo(end_effector, dest, gain=gain, threshold=treshold)     # TODO question par rapport a autres code avec cp_servo ?? d'où ça vient
                # qd = jacobian_i_k_optimisation(robot, v, qd_max=qd_max)[1]                              # TODO question par raport à v, est ce la vitesse du end effector?
               
                # velocity of the end effector
                velocity = (data.body('link6').xpos - pos_nm1)*(1/1e-2)
                # velocity vector
                velocityV = [velocity[0], velocity[1], velocity[2], 0, 0, 0]
                pos_nm1 = (data.body('link6').xpos).copy()
               
                qd = jacobian_i_k_optimisation(robot, velocityV, qd_max=qd_max)[1]

            else:
                qd, arrived = rtb.jp_servo(robot.q, dest, gain=gain, threshold=treshold)

            qpos0 = [qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], 0]
            # kp
            qpos = [x * 2 for x in qpos0]
            data.ctrl = qpos

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(1e-2)

        return arrived, robot.q

'''
Same as robot_move_to but without drake solver
only with roboticstoolbox-python
'''
def move(viewer, robot, position, numberOfSteps = 500):

        robot.q = [     data.joint('joint1').qpos, data.joint('joint2').qpos, 
                        data.joint('joint3').qpos, data.joint('joint4').qpos, 
                        data.joint('joint5').qpos, data.joint('joint6').qpos]
        
        Tep = sm.SE3.Trans(position['x'], position['y'], position   ['z']) * sm.SE3.OA([1, 0,1], [0, 0, -1])
        sol = robot.ik_LM(Tep)         # solve IK

        qt = rtb.jtraj(robot.q, sol[0], numberOfSteps)
        #print(qt.q[numberOfSteps-1])
        for steps in range(numberOfSteps):

            qpos = qt.q[steps]
            data.ctrl = [qpos[0], qpos[1], qpos[2], qpos[3], qpos[4], qpos[5], 0]

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(1e-2)


def crane_move_to(dest, n_sample):
    T_dest = SE3(dest['x'], dest['y'], dest['z'])
    print("t dest", T_dest)
    print("end effector position", model.body('end_effector').pos)
    traj = rtb.ctraj(SE3(model.body('end_effector').pos), T_dest, n_sample)
    
    for i in range(n_sample ):
        print(SE3.Tx(traj[i].x))
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
        time.sleep(1e-2)

lite6 = rtb.models.Lite6()
lite6.base = SE3(4, 0, 0.0)*SE3.Rz(pi/2)

xml_path = 'mainV2.xml'
model = mujoco.MjModel.from_xml_path(xml_path)

data = mujoco.MjData(model)

model.body('link_base').pos = [0.4, 0, 0]
model.body('link_base').quat = [1, 0, 0, 1]


T_pick = SE3(2, 3-0.27, 0.3)
# T_place_up = SE3(0.0, 0.2, 0.1)
# T_place = SE3(0, 0.2, 0.09)
T_place_up = SE3(0.0, 2, 2)
T_place = SE3(0, 2, 0.9)

position = {'x': 0.2, 'y': 0.3, 'z': 0.32}
positionShaft = {'x': 0.2, 'y': 0.285, 'z': 0}

lite6Move = 0
shaftUp = 0
craneMove = 0
takeTheBrick = 0

print(dir(model.body('rope')))
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    i = 0
    while viewer.is_running(): #and i < sim_steps:
        
        # if lite6Move == 0:            # move the lite6
        #     lite6Move = 1
        #     # move(viewer, lite6, position, 500)
        #     robot_move_to(viewer, lite6, position)
        if shaftUp == 0:
            shaftUp = 1
            for i in range(2000):
                shaftPos -= 0.0001
                model.body('moving_box').pos[1] = model.body('beam').pos[1] + shaftPos
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(1e-2)

        if craneMove == 0:
            craneMove = 1
            crane_move_to(positionShaft, 1500)
            
        if takeTheBrick == 0:
            takeTheBrick = 1
            data.ctrl = [0, 0, 0, 0, 0, 0, 0, 1]
            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(1)

            print("down the rope !!")
            for i in range(2000):   # down the rope
                shaftPos += 0.0001
                model.body('moving_box').pos[1] = model.body('beam').pos[1] + shaftPos
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(1e-2)

            time.sleep(1)

            print("up the rope !!")
            for i in range(15000    ):
                shaftPos -= 0.00005  # up the rope   
                model.body('moving_box').pos[1] = model.body('beam').pos[1] + shaftPos
                mujoco.mj_step(model, data)
                viewer.sync()
                time.sleep(1e-2)

        # print(data.body('link6').xpos)  # position of end effector

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(1e-2)
        i +=1
        