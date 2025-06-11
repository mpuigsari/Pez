# core/modem_io.py

import serial
import threading
import time
from queue import Queue, Empty
from typing import Optional


class ModemIOMgr:
    """
    Manages raw serial I/O in a background thread. All incoming bytes get pushed
    into an internal Queue, from which scheduler steps can consume them. 
    """

    def __init__(self, port: str, baud: int = 9600, timeout: float = 0.1):
        self._port = port
        self._baud = baud
        self._timeout = timeout

        # Underlying pyserial Serial instance
        self._ser = serial.Serial(port, baud, timeout=timeout)

        # A queue to hold incoming bytes
        self._rx_queue = Queue()

        # A flag to tell the reader thread to stop
        self._stop_event = threading.Event()

        # Start the reader thread
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

    def _reader_loop(self):
        """Continuously read single bytes and enqueue them until stopped."""
        while not self._stop_event.is_set():
            try:
                data = self._ser.read(1)  # read up to 1 byte (blocking with timeout)
            except Exception:
                break

            if data:
                # data is a bytes object of length 1
                self._rx_queue.put(data)

        # Once stop_event is set or an error occurs, exit thread
        return

    def get_byte(self, timeout: Optional[float] = None) -> Optional[bytes]:
        """
        Pop one incoming byte (as bytes object of length 1) from queue.
        If queue is empty, block up to `timeout` seconds. If no data arrives, return None.
        """
        try:
            raw = self._rx_queue.get(timeout=timeout)
            return raw  # bytes length=1
        except Empty:
            return None

    def send_packet(self, raw: bytes) -> None:
        """Write the given raw bytes to the serial port immediately."""
        if not self._ser.is_open:
            raise RuntimeError("Serial port closed")
        self._ser.write(raw)

    def close(self) -> None:
        """Signal the reader thread to stop, join it, and then close serial port."""
        self._stop_event.set()
        self._reader_thread.join(timeout=1.0)
        try:
            self._ser.close()
        except Exception:
            pass
