# Harmony

The harmony package developed for dynamic obstacle velocity estimation.

## Installation

- Run `rosdep update`
- Run `sudo apt install python3-vcstool`
- Run `vcs import --recursive --input ./moving_object_tracking/harmony.repos`

## Usage

- Create a map using `mapping.launch`
- Save the map using `map_saver` in `map_server` (ensure the `/map` topic is saved)
- Dilate/inflate the map around static obstacles (plus some margin) using a painting program, e. g. `Krita`
- Ensure the modified map is loaded
- Run `localization.launch`, give an initial ICP estimate
- Run `tracking.launch`
- Enjoy!
