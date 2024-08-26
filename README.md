[![Powered by the Spatial Math Toolbox](https://github.com/bdaiinstitute/spatialmath-python/raw/master/.github/svg/sm_powered.min.svg)](https://github.com/bdaiinstitute/spatialmath-python)

# MuJoCo test with Lite6
This repositiory was created for Fari's Brickiebot project with MuJoCo. 

```
cd model
python main.py
```
We tested the code only on Ubuntu 20.04 and windows 10

<br>


## Python dependencies

- numpy==1.26
- mujoco==latest
- spatialmath==latest
- roboticstoolbox-python==[this version](https://github.com/Mordween/robotics-toolbox-python)


## Robot selection 

Uncomment / Comment line 4/5 in ```main.xml  ``` <br>
Uncomment / Comment line 4/5 in ```main.py  ```<br>
Uncomment / Comment line 24/25 in ```parameters.py``` for the robot's position.

<br>

## Using MuJoCo with Lite6 and MyCobot
MuJoCo uses .stl files for loading meshes and XML/MJCF files for displaying robots (typically created in URDF format).

### Lite6
For the Lite6 robot, we found the necessary XML file directly in the <a href="https://github.com/google-deepmind/mujoco_menagerie" target="_blank"> MuJoCo Menagerie GitHub repository </a>.

### MyCobot
For MyCobot, we needed to convert the URDF file to XML (MJCF). This was accomplished using the compile script located in the bin folder.

```
cd bin
./compile [path to your file] [file path]
```

## Implementing a Rope in MuJoCo
In MuJoCo, there are two approaches to creating a rope or a rope-like object:

<b> Rope Object: </b>This allows you to create a rope directly. However, it has several limitations:

It easily passes through rigid bodies (similar to what happens in enable3D).
It cannot be wrapped around a mesh more than once. (in our case)

<b>Tendon:</b> This option is effective when you need a rope-like object to suspend an item since it is not made up of multiple cylinders. However, this solution was not adopted because tendons cannot collide with the rigid body they are attached to if they are not directly fixed to it (making it impossible to pull the rope back up).

In the end, we opted to use the rope object. Although we do not wrap it around anything, we simply pull it over the pulley to raise or lower it.

## Solver Options in MuJoCo
MuJoCo offers three different solvers: Newton, GC, and PGS.

Newton Solver: The main advantage of the Newton solver is its fast convergence (2-3 iterations are typically sufficient). However, from personal observation, it seems to have some issues with physics, particularly with object collisions.

GC Solver: We are currently using the GC solver with 1000 iterations.

## Achieving Real-Time Performance in the Simulator
To create an effective simulator, real-time performance is essential, meaning one second in the real world should correspond to one second in the simulator. Achieving this requires real-time processing.

Given that our primary programming language is Python, an interpreted language rather than a compiled one, its execution speed is significantly slower compared to C/C++.

Currently, we use a simple match statement to implement a state machine. By placing this state machine within a while loop (which runs indefinitely as long as the simulator window is open), we can transition between states sequentially.

At the end of each loop, we synchronize with the MuJoCo viewer, update the simulation state (including model and data), and introduce a relative delay between each loop iteration. This delay is calculated as the difference between the maximum wait time and the actual execution time of the loop.