# pez_docker

Docker setups used throughout the project. Images are built and pushed to Docker Hub under the `mapuigsari/pez` repository.

- **pez/** – fish‑side container (`mapuigsari/pez:core-arm64v8`) for the Raspberry Pi. Provides ROS 2 Humble with Navigator support and can optionally start the acoustic modem bridge.
- **host/** – desktop container (`mapuigsari/pez:core-amd64`) with joystick support, PlotJuggler and all ROS 2 packages prebuilt.
- **comms/** – minimal image (`mapuigsari/pez:comms-arm64v8`) running only the acoustic `pez_comms` node, used when the modem is placed on a separate buoy.

Each subdirectory contains a `Dockerfile`, `entrypoint.sh`, and an example `docker-compose.yml` to launch the container.

