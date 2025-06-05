# pez_core/sensors/sensor_manager.py

import time

from .tsys01 import TSYS01
from .ms5837 import MS5837, MODEL_30BA


class SensorManager:
    """
    Manages TSYS01 and MS5837 sensors on I²C bus 6 using the standalone sensor scripts.
    Each call to a read_* method performs a fresh measurement.
    """

    def __init__(self, use_tsys01: bool = True, use_ms5837: bool = True):
        # Sensor instances (or None if disabled)
        self.tsys01 = None
        self.ms5837 = None

        if use_tsys01:
            try:
                # Instantiate TSYS01 on I²C bus 6
                self.tsys01 = TSYS01(bus=6)
                if not self.tsys01.init():
                    raise RuntimeError("TSYS01.init() returned False")
            except Exception as e:
                raise RuntimeError(f"Failed to initialize TSYS01: {e}")

        if use_ms5837:
            try:
                # Instantiate MS5837 (model 30BA) on I²C bus 6
                self.ms5837 = MS5837(model=MODEL_30BA, bus=6)
                if not self.ms5837.init():
                    raise RuntimeError("MS5837.init() returned False")
            except Exception as e:
                raise RuntimeError(f"Failed to initialize MS5837: {e}")

    def read_tsys01(self) -> float:
        """
        Perform a single temperature measurement on the TSYS01.
        Returns:
            float: Temperature in °C.
        Raises:
            RuntimeError: If TSYS01 was not initialized.
        """
        if not self.tsys01:
            raise RuntimeError("TSYS01 not enabled")
        # Trigger a conversion and read result
        if not self.tsys01.read():
            raise RuntimeError("TSYS01.read() failed")
        return self.tsys01.temperature()

    def read_ms5837_temp(self) -> float:
        """
        Perform a single measurement on MS5837 and return temperature.
        Returns:
            float: Temperature in °C.
        Raises:
            RuntimeError: If MS5837 was not initialized.
        """
        if not self.ms5837:
            raise RuntimeError("MS5837 not enabled")
        # Trigger pressure+temperature conversion
        if not self.ms5837.read():
            raise RuntimeError("MS5837.read() failed")
        return self.ms5837.temperature()

    def read_ms5837_pressure(self) -> float:
        """
        Perform a single measurement on MS5837 and return pressure.
        Returns:
            float: Pressure in mbar.
        Raises:
            RuntimeError: If MS5837 was not initialized.
        """
        if not self.ms5837:
            raise RuntimeError("MS5837 not enabled")
        # Trigger pressure+temperature conversion
        if not self.ms5837.read():
            raise RuntimeError("MS5837.read() failed")
        return self.ms5837.pressure()

    def read_ms5837_depth(self, fluid_density: float = 997.0) -> float:
        """
        Perform a single measurement on MS5837 and return depth relative to MSL.
        Args:
            fluid_density (float): Density in kg/m³ (default: 997 for freshwater).
        Returns:
            float: Depth in meters.
        Raises:
            RuntimeError: If MS5837 was not initialized.
        """
        if not self.ms5837:
            raise RuntimeError("MS5837 not enabled")
        # Trigger pressure+temperature conversion
        if not self.ms5837.read():
            raise RuntimeError("MS5837.read() failed")
        self.ms5837.setFluidDensity(fluid_density)
        return self.ms5837.depth()

    def close(self):
        """
        No explicit bus‐unlocking needed, but we can set instances to None
        if we want to free resources.
        """
        self.tsys01 = None
        self.ms5837 = None
