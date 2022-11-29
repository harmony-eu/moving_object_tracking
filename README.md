# Harmony Moving Object Tracking

**Description:** The harmony package developed for dynamic obstacle velocity estimation.   
**Container:** Humble base container

## Installation

- Run `rosdep update`
- Run `sudo apt install python3-vcstool`
- Run `vcs import --recursive --input ./moving_object_tracking/harmony.repos`

## Usage
<!-- - Create a map using `mapping.launch` -->
<!-- - Save the map using `map_saver` in `map_server` (ensure the `/map` topic is saved) -->
- Dilate/inflate the map around static obstacles (plus some margin) using a painting program, e. g. `Krita`
- Ensure the map and modified map are in the `maps` folder, and follow the naming convention.
- Check if params in `localization.launch.py` and `tracking.launch.py` are correct.
- Run `localization.launch.py` to start the map server (assumes map-to-odom is provided by another node). This launch files takes the `location` parameter (e.g. *abb*, *eth* ...), which is used to get the correct map from the `maps` folder.
- Run `tracking.launch.py`  for object tracking (use `tracking_nmcl.launch.py` if you need to use the scan_merged topic from the nmcl node)
- Run `semantics.launch.py` to launch YOLO node and semantic_tracking node, which project YOLO classes onto the tracked obstacles
- Enjoy!

## Notes
The ROS1 version of this package also had launch files for mapping and localization. Those were not ported (localization is ported but untested and commented-out).