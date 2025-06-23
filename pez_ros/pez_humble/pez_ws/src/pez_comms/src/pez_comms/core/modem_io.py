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

    class ModemIOMgr:
        def __init__(self,
                    port: str,
                    baud: int = 9600,
                    timeout: Optional[float] = 0.1):
            """
            port:        e.g. "/dev/ttyUSB0"
            baud:        e.g. 9600
            timeout:     per‐read timeout in seconds;
                        use None to block until at least 1 byte arrives
                        use 0.0 for completely non‐blocking reads
            """
            self._port    = port
            self._baud    = baud
            self._timeout = timeout

            # open in _exactly_ the right mode:
            self._ser = serial.Serial(
                port=self._port,
                baudrate=self._baud,
                bytesize=serial.EIGHTBITS,      # 8 data bits
                parity=serial.PARITY_NONE,      # no parity
                stopbits=serial.STOPBITS_ONE,   # 1 stop bit
                timeout=self._timeout,          # maps to VMIN/VTIME
                xonxoff=False,                  # no software flow‐control
                rtscts=False,                   # no hardware flow‐control
                dsrdtr=False,                   # no DSR/DTR flow‐control
                exclusive=True                  # Linux: lock the port (pySerial ≥3.4)
            )

            # clear any junk in the buffers
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()

            
            self._rx_queue    = Queue()
            self._stop_event  = threading.Event()
            self._reader_thread = threading.Thread(
                target=self._reader_loop,
                daemon=True
            )
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
