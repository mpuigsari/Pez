# pez_core/sensors/__init__.py

# Expose SensorManager from the sensor_manager.py module
from .sensor_manager import SensorManager

# And if you still need to expose the raw drivers, you can also:
from .tsys01 import TSYS01
from .ms5837 import MS5837, MODEL_30BA

__all__ = [
    'SensorManager',
    'TSYS01',
    'MS5837',
    'MODEL_30BA',
]
