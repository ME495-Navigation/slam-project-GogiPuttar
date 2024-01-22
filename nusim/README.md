# `nusim` package:
The `nusim` package loads a mapped arena with mapped cylindrical obstacles for the nubot.
It also loads one turtlebot.

## Launchfiles

### `nusim.launch.xml`
- Launches `nuturtle_description` package's `load_one.launch.py` to load one `red` turtlebot.
- Runs `rviz` with a specific configuration.
- Runs the `nusim` node to load a world configured by a world yaml file.
