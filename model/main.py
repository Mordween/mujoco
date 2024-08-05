from fonction import *


model = mujoco.MjModel.from_xml_path(param.xml_path)
lite6 = rtb.models.Lite6()

sim = Simulation(model, lite6)



T_pick = SE3(2, 3-0.27, 0.3)
# T_place_up = SE3(0.0, 0.2, 0.1)
# T_place = SE3(0, 0.2, 0.09)
T_place_up = SE3(0.0, 2, 2)
T_place = SE3(0, 2, 0.9)

# position = {'x': 0.2, 'y': 0.3, 'z': 0.32}
position = {'x': 0.2, 'y': 0.3, 'z': 0.030 } 
positionShaft = {'x': 0.2, 'y': 0.295, 'z': 0}      # the value is not 0.3 because you have to consider the radius of the shaft
# 0.285
positionShaft2 = {'x': 0, 'y': 0.09, 'z': 0} # 0.1 - 0.02/2
positionShaft3 = {'x': 0, 'y': 0.18, 'z': 0}

lite6Move = 0
shaftUp = 0
craneMove = 0
takeTheBrick = 0

simulation_action = 'init' 

with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    start = time.time()
    viewer.cam.trackbodyid = 15
    viewer.cam.distance = 1.5
    viewer.cam.lookat = [0, 0, 0]
    viewer.cam.elevation = -45
    viewer.cam.azimuth = 45
    print(dir(model.body('brick')))

    mujoco.mj_step(sim.model, sim.data)
    viewer.sync()

    # print(dir(data.body("gripper_left")))
    #sim.data.body("gripper_left").xipos = [0, 0.1 ,0]
    previous_time = time.time()
    while viewer.is_running(): #and i < sim_steps:
        
        match simulation_action :

            case 'init' :
                # move(viewer, lite6, position, numberOfSteps=500)
                # robot_move_to(viewer, lite6, position)
                simulation_action = 'rope_init'

            case 'rope_init':
                positionZ = 0.16
                if (sim.data.body('gripper_rope').xpos[2]< positionZ):
                    shaftPosUp()
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + param.shaftPos
                else:
                    simulation_action = 'shaftMove'

            case 'shaftMove' :
                sim.crane_move_to(viewer, positionShaft, 1500)
                sim.wait(viewer, 5)
                simulation_action = 'down_rope'

            case 'down_rope':
                positionZ = 0.046
                if (sim.data.body('gripper_rope').xpos[2]> positionZ):
                    shaftPosDown()
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + param.shaftPos
                else:
                    simulation_action = 'take_brick'

            case 'take_brick' :
                sim.wait(viewer, 2)
                sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4],sim.data.ctrl[5],sim.data.ctrl[6], sim.data.ctrl[7],sim.data.ctrl[8], 0, 0.02, -0.02]
                sim.wait(viewer, 2)
                simulation_action = 'up_rope'
            
            case "up_rope":
                positionZ = 0.15
                if (sim.data.body('gripper_rope').xpos[2]< positionZ):
                    shaftPosUp()
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + param.shaftPos
                else:
                    simulation_action = 'shaft_rebase'

            case 'shaft_rebase':
                sim.crane_move_to(viewer, positionShaft2, 1500)
                sim.wait(viewer, 5)
                simulation_action = 'turn_end_effector'
            
            case 'move_robot':
                quat = [1, 0, 0]

                position = {'x':sim.data.body('brick').xpos[0], 
                            'y':sim.data.body('brick').xpos[1]+0.01, 
                            'z':sim.data.body('brick').xpos[2]} 
                 
                sim.move(viewer, lite6, position, quat, numberOfSteps=500)
                sim.wait(viewer, 5)
                simulation_action = 'lite_take'


            case "turn_end_effector":
                
                sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4], 0.92,
                                 0.0045,-0.0045,sim.data.ctrl[8],sim.data.ctrl[9],sim.data.ctrl[10],sim.data.ctrl[11]]
                simulation_action = 'move_robot'
                sim.wait(viewer, 5)

            case "lite_take" :
                sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4], sim.data.ctrl[5],
                                 0.01,-0.01,sim.data.ctrl[8],sim.data.ctrl[9],sim.data.ctrl[10],sim.data.ctrl[11]]
                sim.wait(viewer, 2)
                # quat = [0, 1, 0]
                # position = {'x':sim.data.body('brick').xpos[0], 
                #             'y':sim.data.body('brick').xpos[1], 
                #             'z':sim.data.body('brick').xpos[2]}
                # move(viewer, lite6, position, quat, numberOfSteps=500)
                # translateY(viewer, lite6, -0.04, 500)
                # simulation_action = 'release_brick'
                simulation_action = 'release_brick'
                sim.wait(viewer, 5)
                                   
            case "release_brick" :
                sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4], sim.data.ctrl[5],
                                 sim.data.ctrl[6],sim.data.ctrl[7],sim.data.ctrl[8],sim.data.ctrl[9], 0, 0]
                simulation_action = 'robot_move'

            case 'robot_move':
                sim.wait(viewer, 2)
                sim.crane_move_to(viewer, positionShaft3, 1500)
                sim.wait(viewer, 2)


                # position = {'x': model.body('brick').pos[0], 'y': model.body('brick').pos[1], 'z': model.body('brick').pos[2]} 
                # quat = [model.body('brick').quat[0], model.body('brick').quat[1], model.body('brick').quat[2]]
                # move(viewer, lite6, position, quat, 500)
                simulation_action = 'end'
            
            case 'end' : 
                print("durée de la simulation", round(time.time()-start), " s")
                simulation_action = 'endV2'


        mujoco.mj_step(model,sim.data)
        viewer.sync()
        time.sleep(max(0, param.timeStep-(time.time()-previous_time)))
        previous_time = time.time()
        