from servos import *
from camera import *
from pid_control import PID
import time
import math

clock = time.clock()

class Robot(object):
    """
    A class to manage the functions of a robot for driving and tracking purposes using a camera and servos.
    """

    def __init__(self, thresholds, gain = 10, p=0.2, i=0.01, d=0.05, imax=0.01):
        """
        Initializes the Robot object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain for the PID.
            i (float): Integral gain for the PID.
            d (float): Derivative gain for the PID.
            imax (float): Maximum Integral error for the PID.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)
        self.Pan_PID = PID(p, i, d, imax)

        # Blob IDs
        self.mid_line_id = 0
        self.obstacle_id = 1
        self.l_line_id = 2
        self.r_line_id = 3

        self.scan_direction = 1
        print("Initialised Robot")


    def stage1(self, speed: float, bias = 0) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        print("Running Stage 1")
        while True:
            print('running loop')
            clock.tick()
            blobs, img = self.cam.get_blobs()
            found_mid, found_l, found_r = None, None, None
            if blobs:
                found_mid_index = self.cam.find_blob(blobs, self.mid_line_id)
                found_l_index = self.cam.find_blob(blobs, self.l_line_id)
                found_r_index = self.cam.find_blob(blobs, self.r_line_id)
                if found_mid_index is not None:
                    found_mid = blobs[found_mid_index]
                if found_l_index is not None:
                    found_l = blobs[found_l_index]
                if found_r_index is not None:
                    found_r = blobs[found_r_index]
            if found_mid:
                print("Found Middle")
                image_center = self.cam.w_centre
                pixel_error = found_mid.cx() - image_center
                print("Pixel Error", pixel_error)

                # Normalize error to [-1, 1]
                # (If pixel_error == ±(img.width/2), error_normalized = ±1)
                error_normalized = pixel_error / (image_center)

                # Update PID with the new error
                steering = self.PID.get_pid(error_normalized, 1)

                # Clip steering to [-1,1] just as a safety measure
                steering = max(min(steering, 1.0), -1.0)

                # Drive the robot using the computed steering
                self.drive(speed, steering)
            else:
                if found_l and found_r:
                    print("Found Left and Right")
                    image_center = img.width() / 2.0
                    lane_center = (found_l.cx() + found_r.cx()) / 2.0
                    pixel_error = lane_center - image_center

                    error_normalized = pixel_error / (img.width() / 2.0)
                    steering = self.PID.get_pid(error_normalized, 1)
                    steering = max(min(steering, 1.0), -1.0)
                    self.drive(speed, steering)

                elif found_l:
                    print("Found Left")
                    desired_position = float(img.width()) * 0.8
                    print(desired_position, "desired positio")
                    pixel_error = found_l.cx() - desired_position
                    error_normalized = pixel_error / (img.width() / 2.0)
                    steering = self.PID.get_pid(error_normalized, 1)
                    steering = max(min(steering, 1.0), -1.0)
                    self.drive(speed, steering)

                elif found_r:
                    print("Found Right")
                    desired_position = img.width() * 0.2
                    print(desired_position, "desired positio")
                    pixel_error = found_r.cx() - desired_position
                    error_normalized = pixel_error / (img.width() / 2.0)
                    steering = self.PID.get_pid(error_normalized, 1)
                    steering = max(min(steering, 1.0), -1.0)
                    self.drive(speed, steering)

                else:
                    print("Nothing Found")
                    self.servo.set_speed(0,0)
                    # time.sleep(0.1)

            print('run')
        self.servo.soft_reset()
        return


    def stage2(self, speed: float, bias: float = 0.0) -> None:
        """
        Obstacle detection algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        print("Running Stage 2")

        def _compute_and_drive(pixel_error: float, speed: float, image_width: float):
            """
            Helper function to compute steering using PID and drive the robot.
            Normalizes error to [-1, 1], clips steering to [-1, 1], and calls self.drive().
            """
            # Normalize pixel_error to the range [-1, 1]
            # e.g. if pixel_error == ±(image_width/2), the normalized error == ±1
            error_normalized = pixel_error / (image_width / 2.0)

            # Get steering from PID
            steering = self.PID.get_pid(error_normalized, 0.75)

            # Clip steering
            steering = max(min(steering, 1.0), -1.0)

            # Drive the robot
            self.drive(round(speed,3), round(steering,3))

        while True:
            clock.tick()
            blobs, img = self.cam.get_blobs()

            found_mid, found_l, found_r, obstacle = None, None, None, None

            if blobs:
                found_mid_index = self.cam.find_blob(blobs, self.mid_line_id)
                found_l_index   = self.cam.find_blob(blobs, self.l_line_id)
                found_r_index   = self.cam.find_blob(blobs, self.r_line_id)
                obstacle_index  = self.cam.find_blob(blobs, self.obstacle_id)

                if found_mid_index is not None:
                    found_mid = blobs[found_mid_index]
                if found_l_index is not None:
                    found_l   = blobs[found_l_index]
                if found_r_index is not None:
                    found_r   = blobs[found_r_index]
                if obstacle_index is not None:
                    obstacle  = blobs[obstacle_index]

            if obstacle:
                print("Found Obstacle")
                self.servo.set_speed(0,0)
                continue

            # If we detect a middle line, steer based on it
            if found_mid:
                print("Found Middle")
                image_center = self.cam.w_centre  # or use img.width() / 2
                pixel_error  = found_mid.cx() - image_center
                print("Pixel Error:", pixel_error)
                _compute_and_drive(pixel_error, speed, img.width())

            # Otherwise, if middle is not found, try using left and right lines to center
            else:
                if found_l and found_r:
                    print("Found Left and Right")
                    lane_center = (found_l.cx() + found_r.cx()) / 2.0
                    image_center = img.width() / 2.0
                    pixel_error  = lane_center - image_center
                    _compute_and_drive(pixel_error, speed, img.width())

                elif found_l:
                    print("Found Left")
                    desired_position = img.width() * 0.20
                    pixel_error      = found_l.cx() - desired_position
                    print("Pixel Error (Left):", pixel_error)
                    _compute_and_drive(pixel_error, speed, img.width())

                elif found_r:
                    print("Found Right")
                    desired_position = img.width() * 0.80
                    pixel_error      = found_r.cx() - desired_position
                    print("Pixel Error (Right):", pixel_error)
                    _compute_and_drive(pixel_error, speed, img.width())

                else:
                    # No relevant blobs found; stop or do some fallback
                    print("Nothing Found")
                    self.drive(0.0, 0.0)

        self.servo.soft_reset()
        return


    def stage3(self, speed: float, distance: float) -> None:
        """
        Obstacle distance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        print("Running Stage 3")
        self.servo.soft_reset()
        def _compute_and_drive(pixel_error: float, speed: float, image_width: float):
            """
            Helper function to compute steering using PID and drive the robot.
            Normalizes error to [-1, 1], clips steering to [-1, 1], and calls self.drive().
            """
            # Normalize pixel_error to the range [-1, 1]
            # e.g. if pixel_error == ±(image_width/2), the normalized error == ±1
            error_normalized = pixel_error / (image_width / 2.0)

            # Get steering from PID
            steering = self.PID.get_pid(error_normalized, 0.75)

            # Clip steering
            steering = max(min(steering, 1.0), -1.0)

            # Drive the robot
            self.drive(round(speed,3), round(steering,3))

        while True:
            clock.tick()
            blobs, img = self.cam.get_blobs()

            found_mid, found_l, found_r, obstacle = None, None, None, None

            if blobs:
                found_mid_index = self.cam.find_blob(blobs, self.mid_line_id)
                found_l_index   = self.cam.find_blob(blobs, self.l_line_id)
                found_r_index   = self.cam.find_blob(blobs, self.r_line_id)
                obstacle_index  = self.cam.find_blob(blobs, self.obstacle_id)

                if found_mid_index is not None:
                    found_mid = blobs[found_mid_index]
                if found_l_index is not None:
                    found_l   = blobs[found_l_index]
                if found_r_index is not None:
                    found_r   = blobs[found_r_index]
                if obstacle_index is not None:
                    obstacle  = blobs[obstacle_index]

            # If obstacle is found, handle it (for instance, track or avoid)
            if obstacle:
                print("Found Obstacle")
                obj_distance = self.calculate_distance(obstacle)[0]
                if obj_distance < distance:
                    self.servo.set_speed(0,0)
                    continue

            # If we detect a middle line, steer based on it
            if found_mid:
                print("Found Middle")
                image_center = self.cam.w_centre  # or use img.width() / 2
                pixel_error  = found_mid.cx() - image_center
                print("Pixel Error:", pixel_error)
                _compute_and_drive(pixel_error, speed, img.width())

            # Otherwise, if middle is not found, try using left and right lines to center
            else:
                if found_l and found_r:
                    print("Found Left and Right")
                    lane_center = (found_l.cx() + found_r.cx()) / 2.0
                    image_center = img.width() / 2.0
                    pixel_error  = lane_center - image_center
                    _compute_and_drive(pixel_error, speed, img.width())

                elif found_l:
                    print("Found Left")
                    desired_position = img.width() * 0.20
                    pixel_error      = found_l.cx() - desired_position
                    print("Pixel Error (Left):", pixel_error)
                    _compute_and_drive(pixel_error, speed, img.width())

                elif found_r:
                    print("Found Right")
                    desired_position = img.width() * 0.80
                    pixel_error      = found_r.cx() - desired_position
                    print("Pixel Error (Right):", pixel_error)
                    _compute_and_drive(pixel_error, speed, img.width())

                else:
                    # No relevant blobs found; stop or do some fallback
                    print("Nothing Found")
                    self.drive(0.0, 0.0)
                    # time.sleep(0.1)  # optional small delay if needed
        return


    def stage4(self, speed: float, target_distance: float, target_angle: float) -> None:
        """
        Obstacle distance + orientation algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        print("Running Stage 4")
        current_angle = 0
        self.servo.soft_reset()
        def _compute_and_drive(pixel_error: float, speed: float, image_width: float):
            """
            Helper function to compute steering using PID and drive the robot.
            Normalizes error to [-1, 1], clips steering to [-1, 1], and calls self.drive().
            """
            # Normalize pixel_error to the range [-1, 1]
            # e.g. if pixel_error == ±(image_width/2), the normalized error == ±1
            error_normalized = pixel_error / (image_width / 2.0)

            # Get steering from PID
            steering = self.PID.get_pid(error_normalized, 0.75)

            # Clip steering
            steering = max(min(steering, 1.0), -1.0)

            # Drive the robot
            self.drive(round(speed,3), round(steering,3))

        while True:
            clock.tick()
            blobs, img = self.cam.get_blobs()

            found_mid, found_l, found_r, obstacle = None, None, None, None

            if blobs:
                found_mid_index = self.cam.find_blob(blobs, self.mid_line_id)
                found_l_index   = self.cam.find_blob(blobs, self.l_line_id)
                found_r_index   = self.cam.find_blob(blobs, self.r_line_id)
                obstacle_index  = self.cam.find_blob(blobs, self.obstacle_id)

                if found_mid_index is not None:
                    found_mid = blobs[found_mid_index]
                if found_l_index is not None:
                    found_l   = blobs[found_l_index]
                if found_r_index is not None:
                    found_r   = blobs[found_r_index]
                if obstacle_index is not None:
                    obstacle  = blobs[obstacle_index]

            # finished = False
            # If obstacle is found, handle it (for instance, track or avoid)
            if obstacle:
                print("Found Obstacle")
                obj_distance = self.calculate_distance(obstacle)[0]
                previous_pan_angle = 0
                while obj_distance < target_distance:
                    self.servo.set_speed(0,0)
                    pid_error, pan_angle = self.track_blob(obstacle)
                    blobs, img = self.cam.get_blobs()
                    if blobs:
                        obstacle_index  = self.cam.find_blob(blobs, self.obstacle_id)
                        if obstacle_index is not None:
                            obstacle  = blobs[obstacle_index]

                    if abs(pid_error) < 0.05:
                        error = target_angle - current_angle
                        self.servo.set_speed(0, error*0.01)
                        time.sleep(0.1)
                        print(f"Prev {previous_pan_angle}, Pan Angle: {self.servo.pan_pos}, Current Angle: {current_angle}")
                        current_angle += (self.servo.pan_pos - previous_pan_angle)
                        previous_pan_angle = self.servo.pan_pos
                        if current_angle > target_angle:
                            print("Found")
                            exit()


            # If we detect a middle line, steer based on it
            if found_mid:
                print("Found Middle")
                image_center = self.cam.w_centre  # or use img.width() / 2
                pixel_error  = found_mid.cx() - image_center
                print("Pixel Error:", pixel_error)
                _compute_and_drive(pixel_error, speed, img.width())

            # Otherwise, if middle is not found, try using left and right lines to center
            else:
                if found_l and found_r:
                    print("Found Left and Right")
                    lane_center = (found_l.cx() + found_r.cx()) / 2.0
                    image_center = img.width() / 2.0
                    pixel_error  = lane_center - image_center
                    _compute_and_drive(pixel_error, speed, img.width())

                elif found_l:
                    print("Found Left")
                    desired_position = img.width() * 0.20
                    pixel_error      = found_l.cx() - desired_position
                    print("Pixel Error (Left):", pixel_error)
                    _compute_and_drive(pixel_error, speed, img.width())

                elif found_r:
                    print("Found Right")
                    desired_position = img.width() * 0.80
                    pixel_error      = found_r.cx() - desired_position
                    print("Pixel Error (Right):", pixel_error)
                    _compute_and_drive(pixel_error, speed, img.width())

                else:
                    # No relevant blobs found; stop or do some fallback
                    print("Nothing Found")
                    self.drive(0.0, 0.0)
                    # time.sleep(0.1)  # optional small delay if needed


        return


    def stage5(self, speed: float, bias: float) -> None:
        """
        Obstacle avoidance algorithm - write your own method!

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        self.servo.soft_reset()
        return


    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
        print("Drive: ", drive, " Steering", steering)
        self.servo.set_differential_drive(round(drive, 3), round(steering, 3))
        # time.sleep_ms(500)
        # self.servo.set_speed(0,0)
        # time.sleep_ms(2000)

    def calculate_distance(self, obstacle_blob):
        """
        Calculate the distance from the robot to the detected obstacle, using the bottom center of the bounding box.
        """
        h_cam = 6  # Camera height from the ground in cm
        theta_cam = self.cam.camera_elevation_angle  # Camera downward tilt angle in degrees
        v_fov, h_fov = self.cam.v_fov, self.cam.h_fov  # Camera field of view angles in degrees
        img_width, img_height = sensor.width(), sensor.height()

        # Use bottom center of the bounding box
        bottom_center_x = obstacle_blob.cx()
        bottom_center_y = obstacle_blob.y() + obstacle_blob.h()

        # Calculate vertical angle deviation
        pixel_error_y = self.cam.h_centre - bottom_center_y
        angle_y = (pixel_error_y * v_fov) / img_height
        theta_obstacle = -theta_cam + angle_y

        # Calculate horizontal angle deviation
        pixel_error_x = bottom_center_x - self.cam.w_centre
        angle_horizontal = (pixel_error_x * h_fov) / img_width

        # Compute horizontal distance
        distance = h_cam * math.tan(math.radians(theta_obstacle)) if theta_obstacle > 0 else float('inf')
        actual_distance = distance / math.cos(math.radians(abs(angle_horizontal)))  # Adjusted for horizontal deviation

        print(f"Horizontal deviation: {angle_horizontal:.2f}°, Vertical angle: {theta_obstacle:.2f}°")
        print(f"Horizontal distance to obstacle: {distance:.2f} cm")
        print(f"Total distance to obstacle: {actual_distance:.2f} cm")
        return actual_distance, angle_horizontal

    def track_blob(self, blob) -> None:
        """
        Adjust the camera pan angle to track a specified blob based on its ID.

        Args:
            blob: The blob object to be tracked
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/sensor.width()*self.cam.h_fov)

        pid_error = self.Pan_PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        # Move pan servo to track block
        self.servo.set_angle(pan_angle)
        return pid_error, pan_angle


    def scan_for_blob(self, threshold_idx: int, step = 2, limit = 20) -> None:
        """
        Scans left and right with the camera to find the line.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
            step (int): Number of degrees to pan the servo at each scan step
            limit (int): Scan oscillates between +-limit degrees
        """
        while True:
            # Update pan angle based on the scan direction and speed
            new_pan_angle = self.servo.pan_pos + (self.scan_direction * step)

            # Set new angle
            self.servo.set_angle(new_pan_angle)

            # Check blobs to see if the line is found
            blobs, _ = self.cam.get_blobs_bottom()
            found_idx = self.cam.find_blob(blobs, threshold_idx)
            if found_idx:
                break

            # Check if limits are reached and reverse direction
            if self.servo.pan_pos >= limit or self.servo.pan_pos <= -limit:
                self.scan_direction *= -1


    def debug(self, threshold_idx: int) -> None:
        """
        A debug function for the Robots vision.
        If no block ID is specified, all blocks are detected and debugged.

        Args:
            threshold_idx (int): Index along self.cam.thresholds to find matching blobs
        """
        while True:
            blobs, img = self.cam.get_blobs()
            if threshold_idx is not None:
                found_idx = self.cam.find_blob(blobs, threshold_idx)
            else:
                found_idx = range(len(blobs))

            if found_idx:
                for blob in [blobs[i] for i in found_idx]:
                    img.draw_rectangle(blob.rect())
                    img.draw_string(blob.cx(),blob.cy(),str(blob.code()))

                    angle_err = blob.cx() - self.cam.w_centre

                    print('\n' * 2)
                    print('Code:       ', blob.code())
                    print('X-pos:      ',blob.cx())
                    print('Pan angle:  ', self.servo.pan_pos)
                    print('Angle err:  ', angle_err)
                    print('Angle corr: ', (angle_err-self.servo.pan_pos)/self.servo.max_deg)
                    print('Block size: ', blob.pixels())

                    time.sleep(1)


    def reset(self) -> None:
        """
        Resets the servo positions to their default states and waits.
        """
        self.servo.soft_reset()


    def release(self) -> None:
        """
        Release all servos (no wait).
        """
        self.servo.release_all()
