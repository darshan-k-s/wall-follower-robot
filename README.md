# wall-follower-robot

### 4 Wheel left wall following differential drive robot

### Demo:
![Alt Text](https://i.imgur.com/kmU79kD.gif)
<!-- [Imgur]() -->

'robot_struct' is the package with world, launch, mesh and urdf files.

'logic' is the package with the scripts.

Clone the repository into the /src folder of a pre-existing catkin workspace.

The 'PID_control.py' is the PID control script.

The 'conditional_control.py' is the wall-follow script without PID.

Launching the simulation:
- spawn.launch is the launch file including Rviz and Gazebo.
- The scripts have to run separately and aren't included in the launch file.

Commands to run simulation (Assuming you are in the main directory of the catkin workspace):

## In terminal 1

```
$ catkin_make
$ source devel/setup.bash
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



Contributors:

- Darshan K S
- Sitaraman S
