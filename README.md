# wall-follower-robot

### 4 Wheel left wall following differential drive robot

### Demo:

<img src="https://github.com/Darshan-K-S-work/wall-follower-robot/blob/master/DemoRun.gif" width="602" height="486" />


Clone the repository into the /src folder of a pre-existing catkin workspace.

The `PID_control.py` is the PID control script.

The `conditional_control.py` is the wall-follow script without PID.

Launching the simulation:
- spawn.launch is the launch file including Rviz and Gazebo.
- The scripts have to be run separately and aren't included in the launch file.

Commands to run simulation (Assuming you are in the main directory of the catkin workspace):

## In terminal 1

```
$ source devel/setup.bash
$ catkin_make
$ roslaunch robot_struct spawn.launch
```

## In terminal 2

```
$ source devel/setup.bash


$ rosrun logic PID_control.py

---- OR ----

$ rosrun logic conditional_control.py

```

## Workspace directory Structure
```
------- wallfollower (Catkin Workspace Directory)
      |
      |--- /devel
      |--- /build
      |--- /src
         |
         |--- /build
         |--- /logic (Scripts)
         |--- /robot_struct (Bot structure)
```

## Logic Directory Contents

```
------ /logic (Scripts folder)
     |
     |--- /scripts
        |--- conditional_control.py (Conditional Control Script)
        |--- PID_control.py (PID Script)
        |--- intertias.py (Bot intertia calculator)

```

## robot_struct Directory Contents


```
------ /robot_struct (Bot structure)
     |
     |--- /launch
     |   |
     |   |--- spawn.launch (Robot and world launch file)
     |
     |--- /meshes
     |   |
     |   |--- hokuyo.dae (LIDAR mesh file)
     |
     |--- /urdf
     |   |
     |   |--- materials.gazebo (Gazebo material colour constants)
     |   |--- plugins.gazebo (Gazebo plugin integrations file)
     |   |--- rob.xacro (Xacro file - robot structure definition)
     |
     |--- /world
     |   |
     |   |--- walls.world (Wall Maze world)
     |   |--- line.world (Straight Line world)
        
```

## About the PID control

Check out this video to get a basic idea of PID control theory: 
[PID Control - A brief introduction](https://www.youtube.com/watch?v=UR0hOmjaHp0).

In this project, we are controlling the angular and linear velocities of the robot. Some special situations that occur are taken care of using default values for the response. 

The angular velocity has a standard PID control with the distance from the left wall taken as the main parameter. 

The linear velocity has only proportinal control, taking the distance from left wall as main parameter. Also, when an object is detected to the front of the robot within 4 meters, the control parameter is shifted to the distance from the object in the front.


Contributors:
- Darshan K S
- Sitaraman S
