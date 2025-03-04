from robot import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
    (37, 63, 15, 65, -21, 20),
    (0, 5, -15, 6, -2, 15),
    (25, 50, -19, -4, -43, -2),
    (19, 45, -19, 6, -42, -21)
]



robot = Robot(thresholds, gain=10)

# robot.stage1(0.05)
# robot.stage2(0.05)
# robot.stage3(0.05, 20)
robot.stage4(0.05, 20, 20)

