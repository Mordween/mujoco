# MuJoCo test with Lite6
This repositiory was created for Fari's Brickiebot project with MuJoCo. 
The simulation is in model/test

```
cd model/test
python lite6Control.py
```

We use the [drake](https://drake.mit.edu) library to create the inverse kinematics. 

This library is only available for Linux and MacOS. We used the code only on Ubuntu 20.04

<br>
Currently, the code allows the shaft to be moved towards the brick, and the brick to be attached to the end of the rope.



## Python dependencies

- numpy==1.26
- drake==latest
- mujoco==latest
- spatialmath==latest
- roboticstoolbox-python==[this version](https://github.com/Mordween/robotics-toolbox-python)

