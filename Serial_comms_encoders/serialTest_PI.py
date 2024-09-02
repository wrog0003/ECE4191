# Need to run this first: python3 -m pip install pyserial

import serial
import time

# Setup serial connection
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

# Function to parse data from Arduino
def parse_data(data):
    try:
        count1, count2 = map(int, data.split(','))
        return count1, count2
    except ValueError:
        return None, None

# Main loop
while True:
    try:
        # Read a line from the serial connection
        line = ser.readline().decode('utf-8').strip()
        if line:
            count1, count2 = parse_data(line)
            if count1 is not None and count2 is not None:
                print(f"Encoder 1: {count1}, Encoder 2: {count2}")
        
        # Process data every 1 ms
        time.sleep(0.001)

    except KeyboardInterrupt:
        print("Stopping...")
        break

# Clean up
ser.close()