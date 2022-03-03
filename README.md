# local-planner-research

## Pre-requirements
* ROS noetic
* gazebo11

### ROS installation:
```console
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

sudo apt install curl # if you haven't already installed curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc

sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo apt install python3-rosdep

sudo rosdep init
rosdep update
```

### Gazebo installation
```console
sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'

wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

sudo apt-get update

sudo apt-get install gazebo11
```


## Run simulation

```console
local_planner_research $ catkin_make
local_planner_research $ cd simulation_ws

local_planner_research/simulation_ws $ source devel/setup.sh
local_planner_research/simulation_ws $ roslaunch robot_description spawn.launch
local_planner_research/simulation_ws $ roslaunch gazebo_ros empty_world.launch
```

### Keboard movement
```console
sudo apt-get install ros-noetic-teleop-twist-keyboard
local_planner_research/simulation_ws $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```