import serial
import time

# --- Configuration ---
serialPort = '/dev/ttyUSB0'  # Change to your COM port (e.g., 'COM3' or '/dev/ttyUSB0')
baudRate = 9600

# Command Prefixes
ON_PREFIX = 0x10
OFF_PREFIX = 0x20

# Bitmasks
RED = 0x01
YELLOW = 0x02
GREEN = 0x04
ALL = 0x07  # Red + Yellow + Green

def clear_all(ser):
    """Turns off Red, Yellow, and Green LEDs."""
    ser.write(bytes([OFF_PREFIX | ALL]))

def set_state(ser, mask):
    """Sets the tower to a specific bitmask state."""
    # First, turn off everything to ensure a clean state
    clear_all(ser)
    # Turn on the bits requested in the mask
    ser.write(bytes([ON_PREFIX | mask]))

def cycle_colors(ser, delay=1.0):
    """
    Cycles through all combinations in a logical order:
    Red -> Orange -> Yellow -> Chartreuse -> Green -> Lime -> White
    """
    # Define the sequence of bitmasks
    # 1=Red, 2=Yellow, 4=Green
    sequence = [
        (RED, "Red"),
        (RED | YELLOW, "Orange (Red + Yellow)"),
        (YELLOW, "Yellow"),
        (YELLOW | GREEN, "Chartreuse (Yellow + Green)"),
        (GREEN, "Green"),
        (GREEN | RED, "Lime/Amber (Green + Red)"),
        (RED | YELLOW | GREEN, "White/All (Red + Yellow + Green)")
    ]

    print("Starting color cycle. Press Ctrl+C to stop.")
    try:
        while True:
            for mask, name in sequence:
                print(f"Current Color: {name}")
                set_state(ser, mask)
                time.sleep(delay)
    except KeyboardInterrupt:
        print("\nStopping and cleaning up...")
        clear_all(ser)

if __name__ == '__main__':
    try:
        with serial.Serial(serialPort, baudRate, timeout=1) as mSerial:
            # Brief wait for serial connection to stabilize
            time.sleep(1) 
            cycle_colors(mSerial, delay=0.8)
    except serial.SerialException as e:
        print(f"Error: Could not open serial port {serialPort}. {e}")