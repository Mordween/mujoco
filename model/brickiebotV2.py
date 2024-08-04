import parameters as param

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
position = {'x': 0.2, 'y': 0.3, 'z': 0.030 }              # X and Y axes are reversed 
positionShaft = {'x': 0.2, 'y': 0.285, 'z': 0}
positionShaft2 = {'x': 0, 'y': 0.09, 'z': 0} # 0.1 - 0.02/2
lite6Move = 0
shaftUp = 0
craneMove = 0
takeTheBrick = 0

simulation_action = 'init' 

with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    start = time.time()
    i = 0
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
                sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4],sim.data.ctrl[5],sim.data.ctrl[6], sim.data.ctrl[7],sim.data.ctrl[8], 0, 0.05, -0.05]
                sim.wait(viewer, 5)
                sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4],sim.data.ctrl[5],sim.data.ctrl[6], sim.data.ctrl[7],sim.data.ctrl[8], 0, 0.05, -0.05]
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
                simulation_action = 'move_robot'
            
            case 'move_robot':
                quat = [0, 1, 0]

                position = {'x':sim.data.body('brick').xpos[0]+0.04, 
                            'y':sim.data.body('brick').xpos[1], 
                            'z':sim.data.body('brick').xpos[2]} 
                 
                sim.move(viewer, lite6, position, quat, numberOfSteps=500)
                sim.wait(viewer, 5)
                simulation_action = 'turn_end_effector'


            case "turn_end_effector":
                
                sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4], 1.57,
                                 sim.data.ctrl[6],sim.data.ctrl[7],sim.data.ctrl[8],sim.data.ctrl[9],sim.data.ctrl[10],sim.data.ctrl[11]]
                simulation_action = 'get_closer'
                sim.wait(viewer, 5)

            case "get_closer" :
                # quat = [0, 1, 0]
                # position = {'x':sim.data.body('brick').xpos[0], 
                #             'y':sim.data.body('brick').xpos[1], 
                #             'z':sim.data.body('brick').xpos[2]}
                # move(viewer, lite6, position, quat, numberOfSteps=500)
                # translateY(viewer, lite6, -0.04, 500)
                # simulation_action = 'release_brick'
                simulation_action = 'end'
                sim.wait(20)
                                   
            case "release_brick" :
                positionZ = 0.9
                speed = 0.0001
                if (sim.data.body('gripper_rope').xpos[2]> positionZ):
                    shaftPosDown()
                    model.body('moving_box').pos[1] = model.body('beam').pos[1] + param.shaftPos
                else:
                    simulation_action = 'robot_move'
                    sim.data.ctrl = [sim.data.ctrl[0],sim.data.ctrl[1],sim.data.ctrl[2],sim.data.ctrl[3],sim.data.ctrl[4],sim.data.ctrl[5],sim.data.ctrl[6],sim.data.ctrl[7],sim.data.ctrl[8], 0, 0, 0]

            case 'robot_move':
                sim.wait(viewer, 2)
                # position = {'x': model.body('brick').pos[0], 'y': model.body('brick').pos[1], 'z': model.body('brick').pos[2]} 
                # quat = [model.body('brick').quat[0], model.body('brick').quat[1], model.body('brick').quat[2]]
                # move(viewer, lite6, position, quat, 500)
                simulation_action = 'endV2'
            
            case 'end' : 
                print(sim.data.body('moving_box').xpos)

        # print(data.body('link6').xpos)  # position of end effector

        mujoco.mj_step(model,sim.data)
        viewer.sync()
        time.sleep(max(0, param.timeStep-(time.time()-previous_time)))
        previous_time = time.time()
        