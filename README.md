# ROS Navigation stack local planner benchmark with Turtlebot3 Model

### Requirements: 
- ROS Noetic Desktop Full
- Git
- Ubuntu 20.04

# Build and installation

If the requirements are met, jump to [Install package](#install-package).

### Install ROS:
```
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list
sudo apt install curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install ros-noetic-desktop-full
```

#### Configure ~/.bashrc

`source /opt/ros/noetic/setup.bash`

#### Init rosdep
```
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```
#### Init catkin
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
```
### Install git
`sudo apt install git`

## Install package

### Clone repo
```
cd ~/catkin_ws/src
git clone https://github.com/rureverek/turtlebot3_local_planner_benchmark.git
```
### Install dependencies

`rosdep install turtlebot3_local_planner_sim`

### Build package
```
cd ~/catkin_ws
catkin_make
```
### Configure environment

in ~/.bashrc
```
export TURTLEBOT3_MODEL=waffle_pi
export GAZEBO_PLUGIN_PATH=~/catkin_ws/src/turtlebot3_local_planner_benchmark/plugins
```

# Run the simulation

**DWA Planner: `roslaunch turtlebot3_local_planner_sim run_simulation_dwa.launch scenario:=<name-of-scenario>`**

**TEB Planner: `roslaunch turtlebot3_local_planner_sim run_simulation_teb.launch scenario:=<name-of-scenario>`**

Three scenarios are supported: 

- `Static`
- `Dynamic_simple`
- `Dynamic_complex`

Note: 
- Letter case matters when specifying scenario name in the run command

Dwa init             |  Dwa just before collision
:-------------------------:|:-------------------------:
![halo](/img/rviz/dwa_dynamic_simple00.png)  |  ![](/img/rviz/dwa_dynamic_simple01.png)
Teb init             |  Teb avoiding dynamic obstacle
![](/img/rviz/teb_dynamic_simple00.png)  |  ![](/img/rviz/teb_dynamic_simple_01.png)
