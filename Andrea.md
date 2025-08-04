# Guía rápida de Pez

Esta guía está pensada para usuarios sin experiencia en programación. Explica cómo poner en marcha el sistema de teleoperación del Pez, un robot submarino controlado con ROS 2 y contenedores Docker.

## 1. Conceptos básicos

- **Pez**: el robot que nada bajo el agua. En su Raspberry Pi se ejecuta el contenedor `mapuigsari/pez:core-arm64v8`.
- **Host**: tu ordenador, desde el que manejas el Pez. Utiliza el contenedor `mapuigsari/pez:core-amd64`.
- **Modos**: tanto el Pez como el host comparten tres modos de arranque:
  - `dev` para una consola interactiva,
  - `cable` para teleoperar con el cable USB/serie,
 - `comms` para activar el módem acústico.


Los contenedores están disponibles en [Docker Hub](https://hub.docker.com/r/mapuigsari/pez).

## 2. Usar el Pez sin módem

1. Clona este repositorio en tu ordenador:
   ```bash
   git clone https://github.com/mpuigsari/Pez
   cd Pez/pez_ros/pez_docker/host
   ```
2. Conecta el mando (joystick) al ordenador.
3. Inicia el modo por cable:
   ```bash
   docker compose up pez-cable
   ```
4. Cuando termines, detén los contenedores con `docker compose down`.
5. Para una terminal temporal dentro del contenedor puedes usar:
   ```bash
   docker compose run --rm pez-dev
   ```
6. Usa el joystick para mover el pez y visualiza datos con RQT o PlotJuggler.

### Mapa básico de botones

| Acción               | Botón (Logitech F710) |
|----------------------|-----------------------|
| Iniciar nado         | 7 (Start/Options)     |
| Detener motores      | 6 (Back/Select)       |
| Activar electroimán  | 2 (X)                 |
| Modo neutro          | 4 (Y)                 |

### Ejes principales

| Movimiento                | Eje (Logitech F710)            |
|---------------------------|--------------------------------|
| Velocidad                 | 4 (stick derecho vertical)     |
| Giro izquierda / derecha  | 0 (stick izquierdo horizontal) |
| Subir / bajar             | 1 (stick izquierdo vertical)   |
| Cámara (pan)              | 6 (cruceta horizontal)         |

La configuración completa de botones y ejes se encuentra en
[`joystick_params.yaml`](pez_ros/pez_humble/pez_ws/src/pez_core/config/joystick_params.yaml).


## 3. Usar el Pez con módem acústico

1. En el Pez (Raspberry Pi) arranca el modo de comunicaciones:
   ```bash
   cd Pez/pez_ros/pez_docker/pez
   docker compose up pez-comms
   ```
2. En tu ordenador inicia también el host en modo `pez-comms`:
   ```bash
   cd Pez/pez_ros/pez_docker/pez
   docker compose up pez-comms
   ```

3. Los contenedores lanzarán automáticamente los nodos con la opción de comunicaciones activada.

## 4. Más información

- Cada paquete tiene un README detallado dentro del repositorio.
- Los contenedores Docker se describen en
  [`pez_ros/pez_docker`](pez_ros/pez_docker/README.md).
- Para entender los paquetes principales consulta
  [`pez_core`](pez_ros/pez_humble/pez_ws/src/pez_core/README.md) y
  [`pez_comms`](pez_ros/pez_humble/pez_ws/src/pez_comms/README.md) dentro de
  [`pez_ros/pez_humble`](pez_ros/pez_humble/README.md).

Con estos pasos deberías ser capaz de poner en marcha el robot y empezar a experimentar tanto con conexión directa como con comunicaciones acústicas.
