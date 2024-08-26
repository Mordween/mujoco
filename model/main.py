from fonction import *

model = mujoco.MjModel.from_xml_path(param.xml_path)
robot = rtb.models.Lite6()
# robot = rtb.models.mycobot280()

sim = Simulation(model, robot)

position = {'x': 0.2, 'y': 0.3, 'z': 0.030 } 
positionShaft = {'x': 0.2, 'y': 0.295, 'z': 0}  # 0.285    # the value is not 0.3 because you have to consider the radius of the shaft
positionShaft2 = {'x': 0, 'y': 0.085, 'z': 0}   # same
positionShaft3 = {'x': 0, 'y': 0.18, 'z': 0}

simulation_action = 'init' 

with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
    start = time.time()
    viewer.cam.trackbodyid = 15
    viewer.cam.distance = 1.5
    viewer.cam.lookat = [0, 0, 0]
    viewer.cam.elevation = -45
    viewer.cam.azimuth = 45
    i = 1
    mujoco.mj_step(sim.model, sim.data)
    viewer.sync()

    while viewer.is_running():
        
        match simulation_action :
            case 'init' :
                simulation_action = 'rope_init'

            #-----------------------------------------------------------------------------------------------#
            #  Position the gripper at the top to move it.
            #-----------------------------------------------------------------------------------------------#
            case 'rope_init':
                positionZ = 0.16
                if (sim.data.body('gripper_rope').xpos[2]< positionZ):
                    shaftPosUp(sim)
                else:
                    simulation_action = 'shaftMove'

            #-----------------------------------------------------------------------------------------------#
            # Move the crane to position it over the brick
            #-----------------------------------------------------------------------------------------------#
            case 'shaftMove' :
                sim.crane_move_to(viewer, positionShaft, 1500)
                sim.wait(viewer, 2)
                simulation_action = 'down_rope'

            #-----------------------------------------------------------------------------------------------#
            # Lower the gripper to grab the brick
            #-----------------------------------------------------------------------------------------------#
            case 'down_rope':
                positionZ = 0.046
                if (sim.data.body('gripper_rope').xpos[2]> positionZ):
                    shaftPosDown(sim)
                else:
                    simulation_action = 'take_brick'

            #-----------------------------------------------------------------------------------------------#
            # Take the brick
            #-----------------------------------------------------------------------------------------------#
            case 'take_brick' :
                sim.wait(viewer, 2)
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], sim.data.ctrl[5],
                                 sim.data.ctrl[6], sim.data.ctrl[7], sim.data.ctrl[8], 0, 0.032, -0.032]
                sim.wait(viewer, 1)
                simulation_action = 'up_rope'

            #-----------------------------------------------------------------------------------------------#
            # Position the gripper at the top to move it. 
            #-----------------------------------------------------------------------------------------------#
            case "up_rope":
                positionZ = 0.15
                if (sim.data.body('gripper_rope').xpos[2]< positionZ):
                    shaftPosUp(sim)
                else:
                    simulation_action = 'shaft_rebase'

            #-----------------------------------------------------------------------------------------------#
            # Move the crane to position it over the wall
            #-----------------------------------------------------------------------------------------------#
            case 'shaft_rebase':
                sim.crane_move_to(viewer, positionShaft2, 1500)
                sim.wait(viewer, 2)
                simulation_action = 'move_robot'
            
            #-----------------------------------------------------------------------------------------------#
            # Move the robot to pick up the brick
            #-----------------------------------------------------------------------------------------------#
            case 'move_robot':
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], sim.data.ctrl[5],
                                 0.001, -0.001, sim.data.ctrl[8], sim.data.ctrl[9], sim.data.ctrl[10], sim.data.ctrl[11]]
                quat = [0, 1, 0]

                param.captureFrequency = 5
                position = {'x':sim.data.body('brick').xpos[0], 
                            'y':sim.data.body('brick').xpos[1]-0.15, 
                            'z':sim.data.body('brick').xpos[2]+0.15} 
                
                sim.move(viewer, sim.robot, position, quat, numberOfSteps=100)
                sim.wait(viewer, 2)
                position2 = {'x':sim.data.body('brick').xpos[0], 
                             'y':sim.data.body('brick').xpos[1]-0.1, 
                             'z':sim.data.body('brick').xpos[2]}
                
                sim.move(viewer, sim.robot, position2, quat, numberOfSteps=10)
                sim.wait(viewer, 2)
                simulation_action = 'get_closer'

            #-----------------------------------------------------------------------------------------------#
            # In this case we bring the robot closer to the brick in several small steps because 
            # if we give too many steps, the robot will have strange movements. 
            #-----------------------------------------------------------------------------------------------#
            case "get_closer":
                quat = [0, 1, 0]
                if(position['y']<sim.data.body('brick').xpos[1]):
                    position = {'x':sim.data.body('brick').xpos[0], 
                                'y':sim.data.body('brick').xpos[1]-0.1+0.01*i, 
                                'z':sim.data.body('brick').xpos[2]+0.005}
                    
                    sim.move(viewer, sim.robot, position, quat, numberOfSteps=3)
                    sim.wait(viewer, 0.1)
                    i += 1
                else :
                    param.captureFrequency = 100
                    sim.wait(viewer, 2)
                    simulation_action = 'lite_take'

            #-----------------------------------------------------------------------------------------------#
            # Close the gripper clamps to grip the brick
            #-----------------------------------------------------------------------------------------------#
            case "lite_take" :
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], sim.data.ctrl[5],
                                 0.01, -0.01, sim.data.ctrl[8], sim.data.ctrl[9], sim.data.ctrl[10], sim.data.ctrl[11]]
                sim.wait(viewer, 2)
                simulation_action = 'place_brick'

            #-----------------------------------------------------------------------------------------------#
            # !! Function under construction !!
            # Move the brick with the gripper and robot
            #-----------------------------------------------------------------------------------------------#
            case 'place_brick':
                quat = [0, 1, 0]
                positionZ = 0.078
                if(sim.data.body('brick').xpos[2]> positionZ):    # 0.6+0.3/2 + little offset
                    positionD = {'x': 0,     # how to put end effector position??
                                 'y': 0.1, 
                                 'z': sim.data.body('brick').xpos[2]+0.005-param.up_down_speed}
                    shaftPosDown(sim)
                    sim.move(viewer, sim.robot, positionD, quat, numberOfSteps=2)
    
                else :
                    simulation_action = 'release_brick'

            #-----------------------------------------------------------------------------------------------#
            # Release the brck after it has been properly positioned 
            #-----------------------------------------------------------------------------------------------#                     
            case "release_brick" :
                sim.data.ctrl = [sim.data.ctrl[0], sim.data.ctrl[1], sim.data.ctrl[2], sim.data.ctrl[3], sim.data.ctrl[4], sim.data.ctrl[5],
                                 sim.data.ctrl[6], sim.data.ctrl[7], sim.data.ctrl[8], sim.data.ctrl[9], 0, 0]
                simulation_action = 'crane_away'

            #-----------------------------------------------------------------------------------------------#
            # Moves the crane away
            #-----------------------------------------------------------------------------------------------#
            case 'crane_away':
                sim.wait(viewer, 2)
                sim.crane_move_to(viewer, positionShaft3, 1500)
                sim.wait(viewer, 2)
                simulation_action = 'end'
            
            #-----------------------------------------------------------------------------------------------#
            # we stop data storage and print the simulation duration
            #-----------------------------------------------------------------------------------------------#
            case 'end' : 
                print("durée de la simulation", round(time.time()-start), "s")

                # stop data storage
                param.store_data = False
                simulation_action = 'default'

        # Advance the simulation by one step after performing an action.
        sim.simStep(viewer)
        