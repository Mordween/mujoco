import mujoco
import mujoco.viewer
import time
xml_path = 'test/main.xml'
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# print(model)
# print(dir(model.cam_pos.dot))
print(data)
print('space')
print(dir(data))

# model.cam.lookat[0] = 0
# model.cam.lookat[1] = 0
# model.cam.lookat[2] = 0

model.body('link_base').pos = [1, 1, 0]

with mujoco.viewer.launch_passive(model, data) as viewer:
    start = time.time()
    i = 0
    # while viewer.is_running(): #and i < sim_steps:
    #     # Get a target position from the reference spline using current sim state
    #     qreal = data.qpos[7:]
    #     qDot_real = data.qvel[6:]
    #     qref, qDotref = spline_ref.getReference(qreal, qDot_real, data.time)
    #     data.ctrl = q_ref
    #     mujoco.mj_step(model, data)

    #     # Pick up changes to the physics state, apply perturbations, update options from GUI.
    #     viewer.sync()

    #     # Slow down sim. for visualization
    #     time.sleep(1e-2)
    #     i += 1



time.sleep(100)