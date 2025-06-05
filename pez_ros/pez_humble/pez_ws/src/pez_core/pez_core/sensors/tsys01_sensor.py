# pez_core/sensors/tsys01_sensor.py

import time
import board
import busio
import tsys01

class TSYS01Sensor:
    """
    Wrapper for the TSYS01 temperature sensor (Celsius Sensor R1).
    Uses I²C address 0x77 by default.
    """
    def __init__(self, i2c_bus=None, address: int = 0x77):
        """
        Initialize TSYS01 on the given I²C bus.
        If i2c_bus is None, create a new busio.I2C(board.SCL, board.SDA).
        """
        if i2c_bus is None:
            self._i2c = busio.I2C(board.SCL, board.SDA)
            while not self._i2c.try_lock():
                time.sleep(0.01)
        else:
            self._i2c = i2c_bus

        # Initialize the TSYS01 driver
        self._sensor = tsys01.TSYS01(i2c=self._i2c, address=address)

    def read_temperature(self) -> float:
        """
        Read and return temperature in °C.
        """
        return float(self._sensor.temperature)

    def close(self):
        """
        Unlock I²C if we locked it here.
        """
        try:
            self._i2c.unlock()
        except RuntimeError:
            pass
