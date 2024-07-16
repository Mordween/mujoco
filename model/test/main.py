import mujoco
import mujoco.viewer
import time
from math import pi
import roboticstoolbox as rtb
from spatialmath import SE3, Twist3
from spatialmath.base import *
# from pydrake.solvers import MathematicalProgram, Solve
import numpy as np 
import copy


lite6 = rtb.models.Lite6()
lite6.base = SE3(4, 0, 0.0)*SE3.Rz(pi/2)

xml_path = 'main.xml'
# xml_path.append('assets/ufactory_lite6/lite6.xml')
model = mujoco.MjModel.from_xml_path(xml_path)

# models.append(mujoco.MjModel.from_xml_path(xml_path[1]))

data = mujoco.MjData(model)

model.body('link_base').pos = [0.4, 0, 0]
model.body('link_base').quat = [1, 0, 0, 1]

# print(dir(model.body('link6')))
# print(model.body('link6').ipos)
# print(model.body('link6').pos)


print(dir(data.actuator))

print(dir(data.actuator))

# traj = rtb.ctraj(SE3(model.body('end_effector')), T_dest, n_sample)

# time.sleep(100)
with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    i = 0
    while viewer.is_running(): #and i < sim_steps:
        # # Get a target position from the reference spline using current sim state
        # qreal = data.qpos[7:]
        # qDot_real = data.qvel[6:]
        # qref, qDotref = spline_ref.getReference(qreal, qDot_real, data.time)
        # data.ctrl = q_ref
        # mujoco.mj_step(model, data)

        # # Pick up changes to the physics state, apply perturbations, update options from GUI.
        # viewer.sync()

        # # Slow down sim. for visualization
        # time.sleep(1e-2)
        # i += 1

        # model.joint('shaft').qpos0 += i/100

          # actuator_name = 'joint1'
          # actuator = data.actuator(actuator_name)
          # actuator.ctrl = (i/100)
        
          # data.ctrl['joint1'] = 1

        data.ctrl = [0, 0, 0, 0, 0, 0, 10]

        # print(data.body('link6').xpos)  # position of end effector

        mujoco.mj_step(model, data)
        viewer.sync()
        time.sleep(1e-2)
        i +=1



# time.sleep(100)