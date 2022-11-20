# Build and installation:

## Install ROS:

`sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`

`sudo apt install curl`

`curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -`

`sudo apt update`

`sudo apt install ros-noetic-desktop-full`

### Configure ~/.bashrc

in ~/.bashrc

`source /opt/ros/noetic/setup.bash`

### Init rosdep

`sudo apt install python3-rosdep`

`sudo rosdep init`

`rosdep update`

### Init catkin

`mkdir -p ~/catkin_ws/src`

`cd ~/catkin_ws/`

`catkin_make`

## Install git and clone repo

`sudo apt install git`

`cd ~/catkin_ws/src`

`git clone https://github.com/rureverek/turtlebot3_local_planner_benchmark.git`


### Install dependencies

`rosdep install turtlebot3_local_planner_sim`

## Build package

`cd ~/catkin_ws`

`catkin_make`

## Configure environment

in ~/.bashrc

`export TURTLEBOT3_MODEL=waffle_pi`

`export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/turtlebot3_local_planner_benchmark/plugins`
