# Harmony

The harmony package developed for dynamic obstacle velocity estimation.

## Usage

- Create a map using `mapping.launch`
- Save the map using `map_saver` in `map_server` (ensure the `/map` topic is saved)
- Dilate/inflate the map around static obstacles (plus some margin) using a painting program, e. g. `Krita`
- Ensure the modified map is loaded
- Run `localization.launch`, give an initial ICP estimate
- Run `tracking.launch`
- Enjoy!
