# Harmony

The harmony package developed for dynamic obstacle velocity estimation. This branch was used on ROS Noetic, Ubuntu 20.04.

## Installation

- Clone this repo in your catkin workspace and checkout the ABB branch
- Run `rosdep update`
- Run `sudo apt install python3-vcstool`
- From the src folder of your catkin workspace, run `vcs import --recursive --input moving_object_tracking/harmony.repos`
- `catkin build` your workspace and source it

## Usage

- Create a map using `mapping.launch`
- Save the map using `map_saver` in `map_server` (ensure the `/map` topic is saved)
- Dilate/inflate the map around static obstacles (plus some margin) using a painting program, e. g. `Krita`
- Ensure the modified map is loaded
- Run `localization.launch`, give an initial ICP estimate
- Run `tracking.launch`
- Enjoy!

## Debugging

- If map server is missing: `sudo apt-get install ros-noetic-map-server`
- If python path error occurs: `export PYTHONPATH=$PYTHONPATH:YOUR_CATKIN_WS/src/occupancy_grid_python/src/occupancy_grid_python`
- If you test it with a rosbag, make sure to enable the `--clock` flag when you play the bag, and run `rosparam set use_sim_time True`