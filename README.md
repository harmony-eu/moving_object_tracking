# Harmony

The harmony package developed for dynamic obstacle velocity estimation. This branch was used on ROS Noetic, Ubuntu 20.04.

## Installation

- Clone this repo in your catkin workspace and checkout the ABB branch
- Run `rosdep update`
- Run `sudo apt install python3-vcstool`
- From the src folder of your catkin workspace, run `vcs import --recursive --input moving_object_tracking/harmony.repos`

## Usage

- Create a map using `mapping.launch`
- Save the map using `map_saver` in `map_server` (ensure the `/map` topic is saved)
- Dilate/inflate the map around static obstacles (plus some margin) using a painting program, e. g. `Krita`
- Ensure the modified map is loaded
- Run `localization.launch`, give an initial ICP estimate
- Run `tracking.launch`
- Enjoy!

## Debugging

 sudo apt-get install ros-noetic-map-server

 git clone git@github.com:awesomebytes/occupancy_grid_python.git

 export PYTHONPATH=$PYTHONPATH::/home/harmony_asl/catkin_workspaces/tracking_ws/src/occupancy_grid_python/src/occupancy_grid_python

to run with rosbag:
rosbag play 20220923_abb_image_dataset.bag --clock
rosparam set use_sim_time True