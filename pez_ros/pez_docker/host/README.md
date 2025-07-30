# Host Docker

This directory provides the desktop container used to teleoperate Pez from an x86/AMD64 laptop.
The image is published on Docker Hub as `mapuigsari/pez:core-amd64` and shares the same mode logic as the fish-side container.

## Dockerfile
Builds a ROS 2 Humble environment with joystick libraries, Navigator support and all packages prebuilt.

## Entrypoint scripts
- **entrypoint.sh** – dispatches to `dev`, `cable` or `comms` mode based on the first argument.
- **dev_entrypoint.sh** – minimal wrapper that sources both ROS setups and drops you into `bash -l`.

## Docker Compose
The provided `docker-compose.yml` defines three services:

| Service     | What it runs                    | Typical use                      |
|-------------|---------------------------------|----------------------------------|
| `pez-dev`   | `dev_entrypoint.sh` for a shell | one-off debugging or apt install |
| `pez-cable` | `entrypoint.sh cable`           | USB/serial tele-op               |
| `pez-comms` | `entrypoint.sh comms`           | radio/MAVLink mode               |

Copy this file to your machine and start a mode with `docker compose up pez-cable` (foreground) or `docker compose run pez-dev` for a temporary shell. The compose file pulls the required image automatically. Plug in the joystick and any serial adapters before launching.

