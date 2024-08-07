# GPIO 2 has a pull up resister
from gpiozero import Button, LED
from time import sleep as sl
out = LED(17)
out2 = LED(27)
while True:
    out2.off()
    out.on()
    sl(0.5)
    out.off()
    out2.on()
    sl(0.5)