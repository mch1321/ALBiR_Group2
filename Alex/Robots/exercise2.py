from tuning import PanTuning
from machine import LED
from servos import Servo
from camera import Cam
import time

led = LED("LED_BLUE")
led.on()

thresholds = [
      (42, 55, 65, 76, 30, 66), # Red
      (22, 40, 40, 65, -100, -60), # Blue
]
print(thresholds)
tuning = PanTuning(thresholds, gain =10, p=1.25, i=0.005, d=0.07)

tuning.measure(0.19)


#0.7hz p=1.25, i=0.005, d=0.07
