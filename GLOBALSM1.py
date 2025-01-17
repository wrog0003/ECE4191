#Physical parameters for Milestone 1


from math import pi
# Useful data units: meters
wheelDiameter = 0.1 # diameter of the wheel
'''
The diameter of the wheels (m)
'''
wheelBase = 0.265 # distance between the centre of both wheels 
'''
The distance between the center of the two drive wheels (m)
'''
wheelBaseCircumference = pi*wheelBase # circumference of the wheel
'''
The circumference of the turning circle or the distance that needs to be traveled for a full rotation (m)
''' 
distancePerPulse = wheelDiameter*pi/(74.8*12) # how far the robot can move per pulse of the encoders
'''
The distance that the robot should travel for each pulse of the encoder (m)'''

#PINS
# Motor Left 
motorLa = 23
'''Left motor pin enable A'''
motorLb = 24
'''Left motor pin enable B'''

# Motor Right 
motorRa = 17
'''Right motor pin enable A'''
motorRb = 27
'''Right motor pin enable B'''

# Encoder Left 
motorLcha = 13
'''Left motor encoder pin A'''
motorLchb = 19
'''Left motor encoder pin B'''
# Encoder Right 
motorRcha = 5
'''Right motor encoder pin A'''
motorRchb = 6
'''Right motor encoder pin B'''

# servo pins
servo1 = 12
servo2 = 16
servo3 = 20
servo4 = 21

# button pins
button2 = 10 # Rear limit switch
button3 = 11
button4 = 22

# Front Limit Switch Pin 
button1 = 9