from final_project import *
from machine import LED

led = LED("LED_BLUE")
led.on()

thresholds = [
(45, 59, 0, 11, -34, -19),#blue
(52, 77, 25, 51, -21, 26), #red
(29, 38, -23, -9, -4, 8) #green
]

robot = FinalProject(thresholds, gain=12)

# robot.follow_blue_dot(max_distance=100)

# try:
#     robot.following_blue()
# finally:
#     robot.servo.set_speed(0,0)
#robot.servo.set_angle(0)
#robot.Centering()

#g=robot.following_blue(0.1)

#ang,final=robot.pan()



ang=0
green= None

while True:
    if green is not None:
        break
    robot.turn(-ang)
    ang=0
    robot.Centering()
    ang,final=robot.pan()
    while ang is None:
        ang,final=robot.redo_pan()
    robot.turn(ang)
    if final:
        # Turn 90
        robot.turn(90)
        robot.check_and_move_green(0.05)
        robot.turn(-180)
        robot.check_and_move_green(0.05)
        # Check for green 


    robot.Centering()
    green=robot.following_blue(0.1)
    print(green)


