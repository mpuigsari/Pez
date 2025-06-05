# pez_core/sensors/sensor_manager.py

import time
import board
import busio

from .tsys01_sensor import TSYS01Sensor
from .ms5837_sensor import MS5837Sensor

class SensorManager:
    """
    Manages multiple sensors on a single I²C bus.
    By default, initializes TSYS01 and/or MS5837 depending on flags.
    """
    def __init__(self, use_tsys01: bool = True, use_ms5837: bool = True):
        # Create and lock the I²C bus once
        self._i2c = busio.I2C(board.SCL, board.SDA, bus=6)
        while not self._i2c.try_lock():
            time.sleep(0.01)

        # Initialize wrappers as requested
        self.tsys01: TSYS01Sensor | None = None
        self.ms5837: MS5837Sensor | None = None

        if use_tsys01:
            try:
                self.tsys01 = TSYS01Sensor(i2c_bus=self._i2c, address=0x77)
            except Exception as e:
                raise RuntimeError(f"Failed to initialize TSYS01: {e}")

        if use_ms5837:
            try:
                self.ms5837 = MS5837Sensor(i2c_bus=self._i2c, address=0x76, model="30BA")
            except Exception as e:
                raise RuntimeError(f"Failed to initialize MS5837: {e}")

    def read_tsys01(self) -> float:
        if not self.tsys01:
            raise RuntimeError("TSYS01 not enabled")
        return self.tsys01.read_temperature()

    def read_ms5837_temp(self) -> float:
        if not self.ms5837:
            raise RuntimeError("MS5837 not enabled")
        return self.ms5837.read_temperature()

    def read_ms5837_pressure(self) -> float:
        if not self.ms5837:
            raise RuntimeError("MS5837 not enabled")
        return self.ms5837.read_pressure()

    def read_ms5837_depth(self, fluid_density: float = 997.0) -> float:
        if not self.ms5837:
            raise RuntimeError("MS5837 not enabled")
        return self.ms5837.read_depth(fluid_density)

    def close(self):
        # Close each sensor if it exists, then unlock I²C
        if self.tsys01:
            self.tsys01.close()
        if self.ms5837:
            self.ms5837.close()
        try:
            self._i2c.unlock()
        except RuntimeError:
            pass
