# Guía rápida de Pez

Esta guía está pensada para usuarios sin experiencia en programación. Explica cómo poner en marcha el sistema de teleoperación del Pez, un robot submarino controlado con ROS 2 y contenedores Docker.

## 1. Conceptos básicos

- **Pez**: el robot que nada bajo el agua. En su Raspberry Pi se ejecuta el contenedor `mapuigsari/pez:core-arm64v8`.
- **Host**: tu ordenador, desde el que manejas el Pez. Utiliza el contenedor `mapuigsari/pez:core-amd64`.
- **Comms**: un contenedor opcional (`mapuigsari/pez:comms-arm64v8`) para colocar el módem acústico en una boya y aumentar el alcance.

Los contenedores están disponibles en [Docker Hub](https://hub.docker.com/r/mapuigsari/pez).

## 2. Usar el Pez sin módem

1. Clona este repositorio en tu ordenador:
   ```bash
   git clone https://github.com/mpuigsari/Pez
   cd Pez/pez_ros/pez_docker/host
   ```
2. Conecta el mando (joystick) al ordenador.
3. Inicia el contenedor del host:
   ```bash
   docker compose up -d
   ```
4. Dentro del contenedor, lanza la teleoperación básica:
   ```bash
   ros2 launch pez_core teleop_launch.py
   ```
5. Usa el joystick para mover el pez. Puedes ver datos y gráficas con RQT y PlotJuggler.

## 3. Usar el Pez con módem acústico

1. En el Pez (Raspberry Pi) ejecuta el contenedor añadiendo la opción de comunicaciones:
   ```bash
   docker compose -f pez_ros/pez_docker/pez/docker-compose.yml up -d
   ```
   o bien inicia la imagen directamente con `comms`.
2. En tu ordenador arranca el contenedor del host como en el apartado anterior.
3. Lanza la teleoperación incluyendo la bandera `comms_flag` para activar el puente acústico:
   ```bash
   ros2 launch pez_core teleop_launch.py comms_flag:=true
   ```
4. Si el módem está en una boya (con el contenedor `comms`), usa el lanzamiento especial:
   ```bash
   ros2 launch pez_comms teleopboya_launch.py
   ```

## 4. Más información

- Cada paquete tiene un README detallado dentro del repositorio.
- Los contenedores Docker se describen en `pez_ros/pez_docker/README.md`.
- Para entender los paquetes principales consulta `pez_core` y `pez_comms` en `pez_ros/pez_humble/pez_ws/src`.

Con estos pasos deberías ser capaz de poner en marcha el robot y empezar a experimentar tanto con conexión directa como con comunicaciones acústicas.
