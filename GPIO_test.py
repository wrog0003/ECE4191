# GPIO 2 has a pull up resister
from gpiozero import Button, LED
from time import sleep as sl
out = LED(17)
while True:
    out.on()
    sl(0.5)
    out.off()
    sl(0.5)