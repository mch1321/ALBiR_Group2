   def turn_R(self, angle: int) -> None:
        servo.set_speed(0.1,0)
        time.sleep_ms(int(angle/90*1200))
        servo.soft_reset()

    def turn_L(self, angle: int) -> None:
        servo.set_speed(0,0.1)
        time.sleep_ms(int(angle/90*1200))
        servo.soft_reset()

if __name__ == "__main__":
    servo = Servo()
    servo.turn_L(60)
    servo.turn_R(90)

    servo.soft_reset()
