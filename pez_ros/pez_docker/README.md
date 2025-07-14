# pez_docker

Docker setups used throughout the project. Images are built and pushed to
Docker Hub under the `mapuigsari/pez` repository.

- **pez/** – fish‑side container (`mapuigsari/pez:core-arm64v8`) for the Raspberry Pi. Provides ROS 2 Humble with Navigator support and can optionally start the acoustic modem bridge.
- **host/** – desktop container (`mapuigsari/pez:core-amd64`) with joystick support, PlotJuggler and all ROS 2 packages prebuilt.
- **comms/** – minimal image (`mapuigsari/pez:comms-arm64v8`) running only the acoustic `pez_comms` node, used when the modem is placed on a separate buoy.

Each subdirectory contains a `Dockerfile`, `entrypoint.sh`, and an example
`docker-compose.yml` to launch the container.

Both the fish and host setups expose **three compose services** that reuse the
same image:

| Mode   | Compose service | Purpose                                                    |
| ------ | --------------- | ---------------------------------------------------------- |
| `dev`  | `pez-dev`       | Interactive shell with both ROS setups sourced            |
| `cable`| `pez-cable`     | Launches tele-op over USB/serial tether                   |
| `comms`| `pez-comms`     | Same launch with radio/MAVLink communications enabled     |

Call the desired service via `docker compose run --rm pez-dev` for short-lived
shells or `docker compose up pez-cable`/`pez-comms` for the long-running modes.

