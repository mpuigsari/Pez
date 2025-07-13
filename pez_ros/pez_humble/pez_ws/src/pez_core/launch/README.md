Launch files for `pez_core`.

- `joy_launch.py` – joystick teleoperation with optional sensor nodes and GUI tools. Flags:
  - `display_flag` (true/false) to show RQT and PlotJuggler.
  - `fish_robot` (true/false) to run on the robot or on the host.
  - `comms_flag` (true/false) to start the acoustic bridge using `pez_comms`.
- `teleop_launch.py` – main teleoperation and camera stack.
  - `test_flag` enables simulation mode without sensors or camera.
  - `comms_flag` starts the acoustic modem node.
