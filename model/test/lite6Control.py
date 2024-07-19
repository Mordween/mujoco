import mujoco
import mujoco.viewer
import time
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3, Twist3
import spatialmath as sm
from spatialmath.base import *
from pydrake.solvers import MathematicalProgram, Solve
import numpy as np 
import copy




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
        dest = SE3(position['x'] - data.body('link_base').xpos[0], 
                   position['y'] - data.body('link_base').xpos[1], 
                   position['z'] - data.body('link_base').xpos[2])
        print(robot.fkine(robot.q))

        pos_nm1 = (data.body('link6').xpos).copy()
        while not arrived:

            robot.q = [ data.joint('joint1').qpos, data.joint('joint2').qpos, 
                        data.joint('joint3').qpos, data.joint('joint4').qpos, 
                        data.joint('joint5').qpos, data.joint('joint6').qpos]
            
            end_effector = SE3(data.body('link6').xpos - data.body('link_base').xpos)
             #robot.fkine(robot.q)
            
            if isinstance(dest, SE3) or (isinstance(dest, np.ndarray) and dest.shape==(4,4)):
                v, arrived = rtb.p_servo(end_effector, dest, gain=gain, threshold=treshold)     # TODO question par rapport a autres code avec cp_servo ?? d'où ça vient
                # qd = jacobian_i_k_optimisation(robot, v, qd_max=qd_max)[1]                              # TODO question par raport à v, est ce la vitesse du end effector?
               
                velocity = (data.body('link6').xpos-pos_nm1)*(1/1e-2)
                velocityV = [velocity[0], velocity[1], velocity[2], 0, 0, 0]
                pos_nm1 = (data.body('link6').xpos).copy()
               
                qd = jacobian_i_k_optimisation(robot, velocityV, qd_max=qd_max)[1]
                print("arrived : ", arrived, "qd : ", qd)
            else:
                qd, arrived = rtb.jp_servo(robot.q, dest, gain=gain, threshold=treshold)

            # qpos = [qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], 0]
            qpos0 = [qd[0], qd[1], qd[2], qd[3], qd[4], qd[5], 0]
            qpos = [x * 2 for x in qpos0]


            data.ctrl = qpos

            mujoco.mj_step(model, data)
            viewer.sync()
            time.sleep(1e-2)

        return arrived, robot.q


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



# def crane_move_to(T_dest, n_sample, shaftCenterD = 0.32):
#     traj = rtb.ctraj(SE3(end_effector.T), T_dest, n_sample)
    
#     for i in range(n_sample ):
        
#         crane.T = SE3.Tx(traj[i].x)
#         end_effector.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)
#         shaftLeft.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)*SE3.Tz(3.785)
#         shaftCenter.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)*SE3.Tz(3.785)
#         shaftRight.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y)*SE3.Tz(3.785)
#         cube.T = SE3.Tx(traj[i].x)*SE3.Ty(traj[i].y+shaftCenterD)*SE3.Tz(SE3(cube.T).z)
#         # twist = Twist3.UnitRevolute([1 ,0, 0],[0, traj[i].y, 0.3785], 0)
#         # shaft.T = twist.SE3(traj[i].z/shaft_radius)*shaft.T
#         print("i : ", i)
#         env.step(1/f)
#         time.sleep(1/f)


# def crane_pick_and_place(T_pick, T_place_up, T_place, n_sample):
#     crane_move_to(T_pick, n_sample, 0.32)
#     time.sleep(5)  
#     # for i in range(n_sample):
#     #     twist = Twist3.UnitRevolute([1 ,0, 0],[SE3(shaftCenter.T).x, SE3(shaftCenter.T).y, SE3(shaftCenter.T).z], 0)
#     #     shaftLeft.T = twist.SE3((i/(1000*n_sample))/shaft_radius)*shaftCenter.T
#     #     shaftCenter.T = twist.SE3((i/(1000*n_sample))/shaft_radius)*shaftCenter.T
#     #     shaftRight.T = twist.SE3((i/(1000*n_sample))/shaft_radius)*shaftCenter.T

#     #     env.step()

#     for i in range (70):
#         cube.T = SE3(SE3(cube.T).x, SE3(cube.T).y-(1/1000), SE3(cube.T).z)
#         env.step()

#     for i in range(int((SE3(T_place_up).z-SE3(brick.T).z)*100)):
#         cube.T = SE3(SE3(cube.T).x, SE3(cube.T).y, SE3(cube.T).z + (1/100))
#         env.step()


#     time.sleep(30)
#     crane_move_to(T_place_up, n_sample, 0.25)   

#     robot_move_to(lite6, env, 1/f, T_place_up*SE3.RPY([0, 0, -90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1)
#     robot_move_to(lite6, env, 1/f, T_place*SE3.RPY([0, 0, -90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1, move_brick=True)
#     robot_move_to(lite6, env, 1/f, lite6.qz, gain=2, treshold=0.001, qd_max=1)



lite6 = rtb.models.Lite6()
lite6.base = SE3(4, 0, 0.0)*SE3.Rz(pi/2)

xml_path = 'mainV2.xml'
# xml_path.append('assets/ufactory_lite6/lite6.xml')
model = mujoco.MjModel.from_xml_path(xml_path)

# models.append(mujoco.MjModel.from_xml_path(xml_path[1]))

data = mujoco.MjData(model)

model.body('link_base').pos = [0.4, 0, 0]
model.body('link_base').quat = [1, 0, 0, 1]

# print(dir(model.body('link6')))
# print(model.body('link6').ipos)
# print(model.body('link6').pos)


print(dir(data.joint('joint1')))


# traj = rtb.ctraj(SE3(model.body('end_effector')), T_dest, n_sample)

# time.sleep(100)

T_pick = SE3(2, 3-0.27, 0.3)
# T_place_up = SE3(0.0, 0.2, 0.1)
# T_place = SE3(0, 0.2, 0.09)
T_place_up = SE3(0.0, 2, 2)
T_place = SE3(0, 2, 0.9)

# position = {'x': 0.2, 'y': 0.3, 'z': 0.2}
position = {'x': 0.2, 'y': 0.3, 'z': 0.32}

lite6Move = 0

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    i = 0
    while viewer.is_running(): #and i < sim_steps:
        
        if lite6Move == 0:
            lite6Move = 1
            # move(viewer, lite6, position, 500)
            robot_move_to(viewer, lite6, position)


        # data.ctrl = [0, 0, 0, 0, 0, 0, 10]

        print(data.body('link6').xpos)  # position of end effector


        # robot_move_to(lite6, viewer, 1e-2, T_place_up*SE3.RPY([0, 0, -90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1)
        # mujoco.mj_step(model, data)
        # viewer.sync()
        # time.sleep(1e-2)
        # robot_move_to(lite6, viewer, 1e-2, T_place*SE3.RPY([0, 0, -90], order='xyz', unit='deg'), gain=2, treshold=0.001, qd_max=1, move_brick=True)
        # mujoco.mj_step(model, data)
        # viewer.sync()
        # time.sleep(1e-2)
        # robot_move_to(lite6, viewer, 1e-2, lite6.qz, gain=2, treshold=0.001, qd_max=1)

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(1e-2)
        i +=1



# time.sleep(100)