# ECE4191 6/08/2024 
# Using Raspberry Pi to recieve encoder data and calculate the speed of the motors. 

import RPi.GPIO as GPIO 
import time

# Setup the GPIO Pins to recieve the encoder pulses. 
motor1cha = 5
motor1chb = 6
motor2cha = 16
motor2chb = 26

# Sets the pin number selection to be not based on the phyiscal board pins 
GPIO.setmode(GPIO.BCM)

# Sets up each GPIO Pin as an Input with a pull up resistor which prevents the pins from picking up noise. 
GPIO.setup(motor1cha, GPIO.IN, pull_up_down=GPIO.PUD_UP) 
GPIO.setup(motor1chb, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(motor2cha, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(motor2chb, GPIO.IN, pull_up_down=GPIO.PUD_UP)

# Setting up variables to capture encoder position and speed
encoder_count = 0
last_time = time.time()
speed = 0

# Callback function updates the encoder count based on the changes detected on the encoder pins
def encoder_callback(channel):
    global encoder_count
    if GPIO.input(motor1cha) == GPIO.input(motor1chb):
        encoder_count += 1
    else:
        encoder_count -= 1

# Event detection for encoder pins 
# - GPIO.Both indicates that this callback function is called whenever there is a 
#   rising or falling edge on either or the motor GPIO Pins. 
# - Calls the encoder_callback function when the event occurs. 
GPIO.add_event_detect(motor1cha, GPIO.BOTH, callback=encoder_callback)
GPIO.add_event_detect(motor1chb, GPIO.BOTH, callback=encoder_callback)


time_interval = 1.0  # Time interval in seconds

try:
    while True:
        # Calculate speed every time_interval
        current_time = time.time()
        if current_time - last_time >= time_interval:
            # Speed calculation
            delta_time = current_time - last_time
            speed = (encoder_count / delta_time)  # counts per second
            
            # Convert speed to RPM (assuming 1 count per revolution, adjust as necessary)
            rpm = speed * 60
            
            # Reset the count and time
            encoder_count = 0
            last_time = current_time
            
            # Print the speed
            print(f"Speed: {speed:.2f} counts/sec, RPM: {rpm:.2f}")

        time.sleep(0.01)  # Sleep to prevent excessive CPU usage

except KeyboardInterrupt:
    # Cleanup GPIO pins on exit
    GPIO.cleanup()
