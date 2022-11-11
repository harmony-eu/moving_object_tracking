# Harmony Moving Object Tracking

**Description:** The harmony package developed for dynamic obstacle velocity estimation.   
**Container:** Humble base container

## Installation

- Run `rosdep update`
- Run `sudo apt install python3-vcstool`
- Run `vcs import --recursive --input ./moving_object_tracking/harmony.repos`

## Usage
- Mapping functionality not yet ported
<!-- - Create a map using `mapping.launch` -->
<!-- - Save the map using `map_saver` in `map_server` (ensure the `/map` topic is saved) -->
<!-- - Dilate/inflate the map around static obstacles (plus some margin) using a painting program, e. g. `Krita` -->
- Ensure the modified map is loaded
- Check if params in `localization.launch.py` and `tracking.launch.py` are correct.
- Run `localization.launch.py`, give an initial ICP estimate
- Run `tracking.launch.py`
- Enjoy!
