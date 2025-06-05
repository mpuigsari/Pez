# pez_core/sensors/ms5837_sensor.py

import time
import board
import busio
import ms5837

class MS5837Sensor:
    """
    Wrapper for the MS5837 (Bar30) pressure + temperature sensor.
    Uses I²C address 0x76 by default (model "30BA").
    """
    def __init__(self, i2c_bus=None, address: int = 0x76, model: str = "30BA"):
        """
        Initialize MS5837 on the given I²C bus.
        If i2c_bus is None, create a new busio.I2C(board.SCL, board.SDA).
        model: "30BA" (0–30 bar) or "90BA" (0–90 bar).
        """
        if i2c_bus is None:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            while not self._i2c.try_lock():
                time.sleep(0.01)
        else:
            self._i2c = i2c_bus

        # Create the MS5837 driver
        self._sensor = ms5837.MS5837(model)

        # Initialize via I²C – returns False if no ACK
        if not self._sensor.init_i2c(address):
            raise RuntimeError(f"MS5837 init failed at address 0x{address:02X}")

        # Perform one initial read to load calibration
        self._sensor.read()

    def read_temperature(self) -> float:
        """
        Trigger a new conversion and return temperature in °C.
        """
        self._sensor.read()
        return float(self._sensor.temperature())

    def read_pressure(self) -> float:
        """
        Trigger a new conversion and return pressure in mbar.
        """
        self._sensor.read()
        return float(self._sensor.pressure())

    def read_depth(self, fluid_density: float = 997.0) -> float:
        """
        Trigger a new conversion and return depth (m) given fluid density (kg/m³).
        Default fluid_density=997.0 (fresh water). For seawater, ≈1029.
        """
        self._sensor.read()
        return float(self._sensor.depth(fluid_density))

    def close(self):
        """
        Unlock I²C if we locked it here.
        """
        try:
            self._i2c.unlock()
        except RuntimeError:
            pass
