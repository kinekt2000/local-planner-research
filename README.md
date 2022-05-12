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
local_planner_research $ cd simulation_ws
local_planner_research $ catkin_make

local_planner_research/simulation_ws $ source devel/setup.sh

# map can be <empty, zig-zag, corner, outdoor>
local_planner_research/simulation_ws $ roslaunch robot_gazebo robot_gazebo map:=empty
```

### Keyboard movement
```console
# after simulation run
sudo apt-get install ros-noetic-teleop-twist-keyboard
local_planner_research/simulation_ws $ rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### Go to the point
```console
# after simulation run
local_planner_research/catkin_ws $ catkin_make
local_planner_research/catkin_ws $ source ./devel/setup.bash
local_planner_research/catkin_ws $ rosrun straight_spotter straight_spotter.py
```
Be carefully, algorithm is absolutely stupid.

## Run local-planners

```console
local_planner_research $ cd catkin_ws
local_planner_research/catkin_make $ catkin_make

local_planner_research/catkin_make $ source devel/setup.sh

# availble arguments (can be undefined): map, local_planner, autogoal
# map           -- map which used to build global plan. Available <zig-zag, corner, outdoor>
# local_planner -- approach to build motion trajectory. Available <dwa, tr, teb, mpc>
# auto_goal     -- flag for automatically set goal pose and start recording. Available <false, true>

local_planner_research/catkin_make $ roslaunch nav_2d robot_navigation
```
