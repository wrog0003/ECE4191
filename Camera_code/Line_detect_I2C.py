import smbus
import time
import RPi.GPIO as GPIO

# Define I2C bus (1 for Raspberry Pi 3 and later)
bus = smbus.SMBus(1)

# TCS34725 address
TCS34725_ADDRESS = 0x29

# TCS34725 registers
TCS34725_ENABLE = 0x00
TCS34725_ATIME = 0x01
TCS34725_CONTROL = 0x0F
TCS34725_ID = 0x12
TCS34725_CDATAL = 0x14  # Clear channel data
TCS34725_RDATAL = 0x16  # Red channel data
TCS34725_GDATAL = 0x18  # Green channel data
TCS34725_BDATAL = 0x1A  # Blue channel data

# Command bit
TCS34725_COMMAND_BIT = 0x80

# Gain settings
TCS34725_GAIN_1X = 0x00
TCS34725_GAIN_4X = 0x01
TCS34725_GAIN_16X = 0x02
TCS34725_GAIN_60X = 0x03

# Functions to read/write to registers
def read_register(reg):
    return bus.read_byte_data(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg)

def write_register(reg, value):
    bus.write_byte_data(TCS34725_ADDRESS, TCS34725_COMMAND_BIT | reg, value)

def read_word(reg):
    low = read_register(reg)
    high = read_register(reg + 1)
    return (high << 8) | low

# Initialize the sensor
def tcs34725_init():
    device_id = read_register(TCS34725_ID)
    if device_id != 29:  # Device ID of TCS34725 is 0x44
        raise Exception("TCS34725 not found")
    
    # Power ON and enable ADC
    write_register(TCS34725_ENABLE, 0x01)  # Power ON
    time.sleep(0.01)
    write_register(TCS34725_ENABLE, 0x03)  # Enable ADC
    write_register(TCS34725_ATIME, 0xFF)  # Integration time
    write_register(TCS34725_CONTROL, TCS34725_GAIN_1X)  # Gain

# Read color values
def tcs34725_get_raw_data():
    r = read_word(TCS34725_RDATAL)
    g = read_word(TCS34725_GDATAL)
    b = read_word(TCS34725_BDATAL)
    c = read_word(TCS34725_CDATAL)
    return r, g, b, c

# Main function
if __name__ == "__main__":
    print('In main')
    tcs34725_init()
    print('In main')
    time.sleep(0.7)  # Wait for the sensor to collect data

    try:
        while True:
            r, g, b, c = tcs34725_get_raw_data()
            print(f"Red: {r}, Green: {g}, Blue: {b}, Clear: {c}")
            time.sleep(1)
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Program terminated")