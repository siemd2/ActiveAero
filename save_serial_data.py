import serial
import time

# Configuration
SERIAL_PORT = "/dev/cu.usbmodem1432401" 
BAUD_RATE = 115200                   
OUTPUT_FILE = "arduino_output.txt"   

try:
    # Connect to Arduino
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE)
    print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

    # Open the file for writing
    with open(OUTPUT_FILE, "w") as file:
        print(f"Logging data to {OUTPUT_FILE}... (Press Ctrl+C to stop)")
        while True:
            if ser.in_waiting > 0:
                # Read and decode incoming data
                data = ser.readline().decode('utf-8').strip()
                print(data)  # Display in console
                file.write(data + "\n")  # Save to file
except KeyboardInterrupt:
    print("\nLogging stopped.")
finally:
    if ser.is_open:
        ser.close()
    print(f"Data saved to {OUTPUT_FILE}.")
