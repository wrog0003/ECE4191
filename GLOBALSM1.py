#Physical parameters for Milestone 1


from math import pi
# Useful data units: meters
wheelDiameter = 0.054 # diameter of the wheel
'''
The diameter of the wheels (m)
'''
wheelBase = 0.22 # distance between the centre of both wheels 
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