# Navigation Strategies for Passively Articulated Mobile Robots Based on Path Planners Existing in ROS

This repository is a fork from https://github.com/Team-FloriBot/FloriBot4.0 and contains the navigation strategies that I have developed during my master's thesis with the above topic. The project is mainly based on:
- mobile robot prototype: FloriBot 4.0 (https://www.hs-heilbronn.de/de/floribot)
- Simulator: CoppeliaSim (https://www.coppeliarobotics.com/)

## What are the most important branches and their function?
- Using physical robot prototype
  - https://github.com/moritzboeker/FloriBot4.0/tree/feature/navigation_real_robot - ROS navigation stack for physical FloriBot 4.0 (running on PC connected to FloriBot 4.0)
  - https://github.com/Team-FloriBot/FloriBot4.0/tree/base_node - Kinematics and other prerequisites for FloriBot 4.0 (running on FloriBot 4.0 itself)
- Using simulated robot model
  - https://github.com/moritzboeker/floribotScenesModels/tree/master - Simulated FloriBot 4.0 model and simulated worlds
  - https://github.com/Team-FloriBot/CoppeliaSimPluginROS - Compiled CoppeliaSim plugin supporting custom ROS messages for FloriBot 4.0
  - https://github.com/moritzboeker/FloriBot4.0/tree/feature/navigation - ROS navigation stack and kinematics
- Navigation approaches for passively articulated robots
  - https://github.com/moritzboeker/FloriBot4.0/tree/feature/navigation_approach1_switch_axesFront_axesRear - 1st Approach: Switching nav_base Between the Front and Rear Carriage (see chapter 4.8.1 of master's thesis)
  - https://github.com/moritzboeker/FloriBot4.0/tree/feature/navigation_approach2_switch_axesFront_POSaxesFrontROTaxesRear - 2nd Approach: Composing nav_base with the Position of the Front and the Orientation of the Rear Carriage (see chapter 4.8.2 of master's thesis)
  - https://github.com/moritzboeker/FloriBot4.0/tree/feature/navigation_approach3_switch_jointFront_jointRear - 3rd Approach: Navigating the Robot at its Articulated Joint (see chapter 4.8.3 of master's thesis)

## How to install?
### ROS Navigation Prerequisites
Install 
Ubuntu 20.04 LTS
ROS Noetic: http://wiki.ros.org/noetic/Installation/Ubuntu 
git
Setup ROS workspace and go into it
$ mkdir -p ~/floribot_ws/src
$ cd ~/floribot_ws/src
$ source /opt/ros/noetic/setup.bash
Download Floribot4.0 Repository from https://github.com/Team-FloriBot/FloriBot4.0
$ git clone git@github.com:Team-FloriBot/FloriBot4.0.git
$ git checkout feature/navigation
$ git submodule update --init --recursive
$ cd ~/floribot_ws
$ catkin_make (will produce errors, but no problem simply proceed installing the packages further down)
$ source devel/setup.bash
Install requirements for teb_local_planner
$ sudo apt install python3-rosdep2
$ rosdep install teb_local_planner
Install further ROS packages
$ sudo apt install ros-noetic-map-server
$ sudo apt install ros-noetic-pcl-ros
$ sudo apt install ros-noetic-tf2-sensor-msgs
$ sudo apt install ros-noetic-move-base-msgs
$ sudo apt install ros-noetic-rviz
$ sudo apt install ros-noetic-rqt-reconfigure
$ sudo apt install ros-noetic-rqt-tf-tree
$ sudo apt install ros-noetic-rosbash
$ sudo apt-get install ros-noetic-cmake-modules
$ sudo apt-get install ros-noetic-filters
$ sudo apt-get install ros-noetic-sbpl
$ sudo apt-get install ros-noetic-sbpl-lattice-planner
$ sudo apt-get install ros-noetic-mbf-costmap-core
$ sudo apt-get install ros-noetic-mbf-msgs
$ rosdep install teb_local_planner
$ sudo apt-get install gmapping
$ rosdep install --from-paths src --ignore-src -r -y
Activate costmap_converter plugin for teb_local_planner
$ rospack plugins --attrib=plugin costmap_converter
Check again if workspace builds - if not, install necessary requirements
$ cd ~/floribot_ws
$ catkin_make
### CoppeliaSim Installation
Download CoppeliaSim from https://coppeliarobotics.com/downloads
Extract and rename as CoppeliaSim
Move CoppeliaSim folder to ~/
Download ROSplugin with custom base/Wheels and base/Angles messages
$ git clone git@github.com:Team-FloriBot/CoppeliaSimPluginROS.git 
Move file libsimExtROS.so to ~/CoppeliaSim/
Download floribotScenesModels Repository from https://github.com/moritzboeker/floribotScenesModels/tree/master
$ cd ~/CoppeliaSim/
$ git clone git@github.com:moritzboeker/floribotScenesModels.git
$ cd floribotScenesModels/
$ git checkout master
Launch everything
Terminal 1
$ roscore
Terminal 2
$ cd ~/CoppeliaSim
$ ./coppeliaSim.sh
CoppeliaSim Window
File -> Open Scene -> ~/CoppeliaSim/floribotScenesModels/scenes/floribot_scenes/floribot_playground.ttt
Simulation -> Start Simulation
#### If Plugin is loaded succesfully (see below), you are done
$ ./coppeliaSim.sh
…
[CoppeliaSim:loadinfo]   plugin 'ROS': loading…
[CoppeliaSim:loadinfo]   plugin 'ROS': load succeeded.
…
#### If the Plugin could not be loaded (see below), please proceed by compiling the plugin yourself

Recompile plugin by yourself (according to tutorials from
https://www.coppeliarobotics.com/helpFiles/en/ros1Tutorial.htm
https://ilias.hs-heilbronn.de/goto.php?target=wiki_210629_Simulation )
Create new ROS workspace only to build the plugin
$ mkdir -p ~/plugin_ws/src
$ cd ~/plugin_ws/src/
$ git clone --recursive git@github.com:Team-FloriBot/simExtROS.git
$ cd simExtROS
$ git checkout floribot 
The Floribot 4.0 uses ROS messages which are not yet included into the plugin - therefore we need to manually insert them (this is already done in branch floribot and is only explained below for understanding purposes)
Open ~/plugin_ws/src/sim_ros_interface/meta/messages.txt
Add base/Wheels, base/Angle as two last lines of the document
Open ~/plugin_ws/src/sim_ros_interface/package.xml
Add <depend>base</depend> below other dependencies
Open ~/plugin_ws/src/sim_ros_interface/CMakeLists.txt
Add base within set(PKG_DEPS)
Then we can build the plugin. To do so, we need to install some additional packages
$ cd ~/plugin_ws
$ export COPPELIASIM_ROOT_DIR=~/CoppeliaSim/
$ source ~/floribot_ws/devel/setup.bash
$ sudo apt-get install python3-catkin-tools
$ sudo apt-get install python3-pip
$ pip install xmlschema
$ sudo apt install xsltproc
$ catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
Afterwards the plugin recently built is found in plugin_ws/devel/lib/libsimExtROS.so
Copy the plugin into CoppeliaSim folder
$ cp ~/plugin_ws/devel/lib/libsimExtROS.so ~/CoppeliaSim/
