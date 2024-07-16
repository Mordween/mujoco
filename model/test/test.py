# print(model)
# print(dir(model.cam_pos.dot))
print(data)
print('space')
print(dir(data))

# model.cam.lookat[0] = 0
# model.cam.lookat[1] = 0
# model.cam.lookat[2] = 0

print(model)
print("space")
print(dir(model))
print(model.joint('shaft').qpos0)




print(dir(model.body))
print(dir(data.body))

print(dir(model.joint('shaft').qpos0))

print(dir(data.joint("joint1")))

# model.joint("joint1")

# model.body('link_base').size = [0.5, 0.5, 0.5]
# mujoco.viewer.launch_passive(model, data)