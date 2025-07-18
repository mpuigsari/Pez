# Comms Docker

Minimal Docker image for running only the `pez_comms` node. It is useful when the acoustic modem is placed on a dedicated buoy or any device separate from the main robot. The image is published as `mapuigsari/pez:comms-arm64v8`.

## Usage

The provided `docker-compose.yml` exposes two services:

| Service          | Command                    | Purpose                               |
|------------------|----------------------------|---------------------------------------|
| `pez-comms-dev`  | `dev`                      | Interactive shell with ROS 2 sourced  |
| `pez-comms`      | `pez`                      | Launch the comms bridge for the PEZ   |
| `bluerov-comms`  | `bluerov`                  | Launch the BlueROV2 comms variant     |

Start a container with:

```bash
docker compose up pez-comms
```

or open a shell for debugging with:

```bash
docker compose run --rm pez-comms-dev
```

The entrypoint internally calls `ros2 launch pez_joy pez_launch.py comms_flag:=true` with the appropriate robot argument.
