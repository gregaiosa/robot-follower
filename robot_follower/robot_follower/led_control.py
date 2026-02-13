import serial
import threading
import time

class LedControl:
    # Command Prefixes
    ON_PREFIX = 0x10
    OFF_PREFIX = 0x20

    # Bitmasks
    RED = 0x01
    YELLOW = 0x02
    GREEN = 0x04
    ALL = 0x07  # Red + Yellow + Green

    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._blink_thread = None

        try:
            self.ser = serial.Serial(self.port, self.baud_rate, timeout=1)
            # Brief wait for serial connection to stabilize
            time.sleep(0.1)
        except serial.SerialException as e:
            print(f"Error: Could not open serial port {self.port}. {e}")

    def _write_bytes(self, data):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(data)
            except serial.SerialException as e:
                print(f"Serial write error: {e}")

    def _set_state(self, mask):
        """Internal method to set the hardware state."""
        # First, turn off everything to ensure a clean state
        self._write_bytes(bytes([self.OFF_PREFIX | self.ALL]))
        # Turn on the bits requested in the mask
        if mask:
            self._write_bytes(bytes([self.ON_PREFIX | mask]))

    def set_color(self, color_mask, blink_ms=0):
        """
        Sets the LED color and optionally blinks.

        :param color_mask: Bitmask of the color(s) to turn on.
        :param blink_ms: Time in milliseconds for the blink interval (on time).
                         If 0 (default), the LED is solid on (infinite blink).
        """
        # Stop any existing blinking thread
        self._stop_event.set()
        if self._blink_thread is not None:
            self._blink_thread.join()
        
        self._stop_event.clear()

        if blink_ms <= 0:
            # Solid on
            with self._lock:
                self._set_state(color_mask)
        else:
            # Start blinking in a separate thread
            self._blink_thread = threading.Thread(
                target=self._blink_loop,
                args=(color_mask, blink_ms / 1000.0)
            )
            self._blink_thread.daemon = True
            self._blink_thread.start()

    def _blink_loop(self, color_mask, interval_s):
        while not self._stop_event.is_set():
            # On
            with self._lock:
                self._set_state(color_mask)
            if self._stop_event.wait(interval_s):
                break
            
            # Off
            with self._lock:
                self._set_state(0)
            if self._stop_event.wait(interval_s):
                break

    def close(self):
        """Clean up resources."""
        self._stop_event.set()
        if self._blink_thread:
            self._blink_thread.join()
        if self.ser:
            # Turn off LEDs before closing
            self._set_state(0)
            self.ser.close()