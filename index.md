## Quintic Walk

This page aggrigates all information to the Hamburg Bit-Bots Quintic Walk software that is a simple-to-use walk controller for non-parallel bipedal robots.
If you find any issues in the following documentation or are not able to run the code, please open an [issue](https://github.com/bit-bots/quintic_walk/issues).

### Video
TODO add video here

### Tutorial

#### Building (ROS 2 Version)
You will need to have [ROS 2 rolling](https://docs.ros.org/en/rolling/Installation.html) and [colcon](https://docs.ros.org/en/rolling/Tutorials/Colcon-Tutorial.html) installed. See the linked documentation for more information. The ROS 1 version is similar, but you will need to checkout the "ROS 1" tag in the gits and use [catkin](http://wiki.ros.org/catkin) to build the software.

Create a colcon workspace:
```
source /opt/ros/rolling/setup.bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```
Download the code
```
git clone https://github.com/bit-bots/bitbots_motion.git
```
Download dependencies
```
git clone https://github.com/ros-sports/biped_interfaces.git
git clone https://github.com/SammyRamone/bio_ik.git
git clone https://github.com/bit-bots/bitbots_tools.git
git clone https://github.com/bit-bots/bitbots_msgs.git
git clone https://github.com/bit-bots/bitbots_misc.git
git clone https://github.com/bit-bots/humanoid_league_msgs.git
git clone https://github.com/bit-bots/ros2_python_extension.git
```

Resolve further dependencies automatically
```
cd ~/dev_ws 
rosdep install -i --from-path src --rosdistro rolling -y
```

Build the walk package
```
colcon build --symlink-install --packages-up-to bitbots_quintic_walk
```

The walk software package is now build. You can perform the following steps if you want to compile the simulator package, the parameter optimization package and the different robot packages too.

```
cd ~/dev_ws/src
git clone https://github.com/bit-bots/wolfgang_robot.git
git clone https://github.com/bit-bots/parallel_parameter_search.git
git clone https://github.com/bit-bots/humanoid_robots_ros2.git

cd ~/dev_ws
colcon build --symlink-install --packages-up-to wolfgang_webots_sim
colcon build --symlink-install --packages-up-to parallel_parameter_optimization
colcon build --symlink-install --packages-up-to bez_moveit_config chape_moveit_config mrl_hsl_moveit_config nao_moveit_config nugus_moveit_config op3_moveit_config rfc_moveit_config  robotis_op2_moveit_config wolfgang_moveit_config
```

#### Running in ROS 2
You will need to source the colcon workspace where you have build the walking
```
source ~/dev_ws/install/setup.bash
```
There are different options on how the walk controller can be started.
On a real robot (this will try to start our hardware interface too)
```
ros2 launch bitbots_quintic_walk test.launch robot_type:=wolfgang
```
In a simulation (this requires Webots in the [RoboCup Humanoid League Virtual Season version](https://github.com/RoboCup-Humanoid-TC/webots/tree/release/projects/samples/contests/robocup) to be installed)
```
ros2 launch bitbots_quintic_walk test.launch sim:=true robot_type:=wolfgang
```
And in a second terminal
```
source ~/dev_ws/install/setup.bash
source ~/dev_ws/src/wolfgang_webots_sim/scripts/setenvs.sh
ros2 launch wolfgang_webots_sim simulator.launch camera:=false robot_type:=wolfgang
```
Just as a visualization in RViz
```
ros2 launch bitbots_quintic_walk viz.launch robot_type:=wolfgang
```
You can specify the robot type that you want to use by changing `wolfgang` to one of `{bez, chape, mrl_hsl, nao, nugus, op3, rfc, robotis, wolfgang}` 

You can use teleop to control the robot 
```
ros2 run bitbots_teleop teleop_keyboard
```

#### Direct Code Interfaces
You can directly call the code either in C++ or in Python.
The Python interface is created using pybind11 and automatically generated during the build process.
You can use it like this:
```
from ament_index_python import get_package_share_directory
from bitbots_quintic_walk_py.py_walk import PyWalk
from bitbots_utils.utils import load_moveit_parameter, get_parameters_from_ros_yaml

# load moveit config values
moveit_parameters = load_moveit_parameter("ROBOT_NAME")

# load walk params
walk_parameters = get_parameters_from_ros_yaml("walking",
                                               f"{get_package_share_directory('bitbots_quintic_walk')}"
                                               f"/config/CONFIG_THAT_YOU_WANT.yaml",
                                               use_wildcard=True)
# create the walk controller
walk_controller = PyWalk("NAMESPACE", walk_parameters + moveit_parameters)

while True:   
    # here you would need to get the sensor information from your robot
    
    # call the walk controller with provided sensor data
    joint_command_msg = walk_controller.step(time_delta, 
                                       speed_command_msg,
                                       imu_msg,
                                       joint_state_msg,
                                       pressure_left_msg, pressure_right_msg)
    # apply the joint goal positions on your robot 
    
    # optionally you can publish debug via ROS 2 to visualize it in PlotJuggler                                   
    walk_controller.publish_debug()                                            
```



### Parameter Optimization
The optimized parameters for many robots are already included in the [software package](https://github.com/bit-bots/bitbots_motion/tree/master/bitbots_quintic_walk/config).

If you want to optimize the parameters yourself, you first need to build the packages (see above).
Make sure that the colcon workspace and Webots is sourced in your terminal
```
source ~/dev_ws/install/setup.bash
source ~/dev_ws/src/wolfgang_webots_sim/scripts/setenvs.sh
```

Now you can start the optimization process 
```
python3 ~/dev_ws/src/parallel_parameter_optimization/scripts/optimize_walk.py --type engine --robot bez --sim webots --sampler MOTPE --trials 1000 --storage mysql://root@localhost/example
```
The parameters allow you to specify the type of robot, simulator and sampler, as well as further arguments for Optuna. Use the `--help` option for more details.

We highly recommend to use the [distributed optimization feature of Optuna](https://optuna.readthedocs.io/en/stable/tutorial/10_key_features/004_distributed.html#distributed). This allows you to start the above command multiple times on the same or different computers, thus greatly increasing the speed of the optimization process. Additionally, it makes it possible to resume a study if some problem occured.

### Adding a new Robot Type
If you want to use the approach on another robot, which is not yet supported, you need to do the following things.
1. Create a robot description package containing the URDF (documentation on generating this from .proto models is [here](https://github.com/bit-bots/humanoid_robots_ros2/blob/master/README.md))
2. Create a MoveIt configuration package containing configs for the IK solver (see also [here](https://github.com/bit-bots/humanoid_robots_ros2/blob/master/README.md) for more information)
3. Provide some simulation with this robot model. The easiest way is to add it to the existing Webots interface. Look at the `__init__()` in `wolfgang_webots_sim/webots_robot_controller.py` and add your robot accordingly
4. Compile the new packages
5. Optimize the parameters as shown above
6. Create a new config file in the bitbots_quintic_walk package with the optimized parameters

### Additional Documentation
Some (older) documentation is directly included in the [walk package](https://github.com/bit-bots/bitbots_motion/blob/master/bitbots_quintic_walk/docs/index.rst).
