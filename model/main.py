from fonction import *

model = mujoco.MjModel.from_xml_path(param.xml_path)
lite6 = rtb.models.Lite6()

sim = Simulation(model, lite6)

position = {'x': 0.2, 'y': 0.3, 'z': 0.030 } 
positionShaft = {'x': 0.2, 'y': 0.295, 'z': 0}  # 0.285    # the value is not 0.3 because you have to consider the radius of the shaft
positionShaft2 = {'x': 0, 'y': 0.09, 'z': 0}
positionShaft3 = {'x': 0, 'y': 0.18, 'z': 0}

simulation_action = 'init' 

with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    start = time.time()
    viewer.cam.trackbodyid = 15
    viewer.cam.distance = 1.5
    viewer.cam.lookat = [0, 0, 0]
    viewer.cam.elevation = -45
    viewer.cam.azimuth = 45

    mujoco.mj_step(sim.model, sim.data)
    viewer.sync()

    previous_time = time.time()
    while viewer.is_running():
        
        match simulation_action :
            case 'init' :
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
                sim.wait(viewer, 2)
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
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], sim.data.ctrl[5],
                                 sim.data.ctrl[6], sim.data.ctrl[7], sim.data.ctrl[8], 0, 0.02, -0.02]
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
                sim.wait(viewer, 2)
                simulation_action = 'move_robot'
            
            case 'move_robot':
                quat = [0, 1, 0]

                position = {'x':sim.data.body('brick').xpos[0], 
                            'y':sim.data.body('brick').xpos[1]-0.1, 
                            'z':sim.data.body('brick').xpos[2]+0.1} 
                
                sim.move(viewer, lite6, position, quat, numberOfSteps=100)
                sim.wait(viewer, 2)
                simulation_action = 'get_closer'

            case "get_closer":
                quat = [0, 1, 0]
                position2 = {'x':sim.data.body('brick').xpos[0], 
                             'y':sim.data.body('brick').xpos[1]-0.1, 
                             'z':sim.data.body('brick').xpos[2]}
                
                position3 = {'x':sim.data.body('brick').xpos[0], 
                             'y':sim.data.body('brick').xpos[1], 
                             'z':sim.data.body('brick').xpos[2]}
                
                sim.move(viewer, lite6, position2, quat, numberOfSteps=10)
                sim.wait(viewer, 2)
                sim.move(viewer, lite6, position3, quat, numberOfSteps=10)
                sim.wait(viewer, 2)
                
                simulation_action = 'lite_take'

            case "turn_end_effector":
                
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], 0.92,
                                 0.0045, -0.0045, sim.data.ctrl[8], sim.data.ctrl[9], sim.data.ctrl[10], sim.data.ctrl[11]]
                simulation_action = 'move_robot'
                sim.wait(viewer, 2)

            case "lite_take" :
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], sim.data.ctrl[5],
                                 0.01, -0.01, sim.data.ctrl[8], sim.data.ctrl[9], sim.data.ctrl[10], sim.data.ctrl[11]]
                sim.wait(viewer, 2)
                # quat = [0, 1, 0]
                # position = {'x':sim.data.body('brick').xpos[0], 
                #             'y':sim.data.body('brick').xpos[1], 
                #             'z':sim.data.body('brick').xpos[2]}
                # move(viewer, lite6, position, quat, numberOfSteps=500)
                # translateY(viewer, lite6, -0.04, 500)
                # simulation_action = 'release_brick'
                simulation_action = 'release_brick'
                sim.wait(viewer, 1)
                                   
            case "release_brick" :
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], sim.data.ctrl[5],
                                 sim.data.ctrl[6], sim.data.ctrl[7], sim.data.ctrl[8], sim.data.ctrl[9], 0, 0]
                simulation_action = 'robot_move'

            case 'robot_move':
                sim.wait(viewer, 2)
                sim.crane_move_to(viewer, positionShaft3, 1500)
                sim.wait(viewer, 2)

                simulation_action = 'end'
            
            case 'end' : 
                print("durée de la simulation", round(time.time()-start), "s")
                simulation_action = 'endV2'


        mujoco.mj_step(model,sim.data)
        viewer.sync()
        time.sleep(max(0, param.timeStep-(time.time()-previous_time)))
        previous_time = time.time()
        