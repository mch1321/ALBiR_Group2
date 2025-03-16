from servos import *
from camera import *
from pid_control import PID
import time
import math

clock = time.clock()

class FinalProject(object):
    def __init__(self, thresholds, gain = 10, p=0.18, i=0.01, d=0.07, imax=0.01):
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
        self.thresholds = thresholds

        # Blob IDs
        self.blue_dot_id = 0
        self.red_cup_id = 1
        self.green_goal = 2

        print("Initialised Robot")

    def following_blue(self, speed=0.1, distance_threshold=70) -> None:
        """
        Follow the blue dot on the screen.
        """
        green=None
        old_y = None

        while True:
            # Get blobs

            frames = 0
            blobs, _ = self.cam.get_blobs()
            blue_dot = self.cam.find_colour_biggest_blob(blobs, self.blue_dot_id)
            blue_dot = self.cam.find_colour_biggest_blob(blobs, self.green_goal)
            while frames < 10 and blue_dot is None:
                self.servo.set_speed(0,0)
                blobs, _ = self.cam.get_blobs()

                # Find the blue dot
                blue_dot = self.cam.find_colour_biggest_blob(blobs, self.blue_dot_id)
                time.sleep(0.02)
                frames += 1

            if blue_dot is not None:

                if old_y is None:
                    green=self.cam.find_colour_biggest_blob(blobs, self.green_goal)
                    old_y = blue_dot.cy()

                current_y = blue_dot.cy()

                if current_y - old_y < -distance_threshold:
                    break

                old_y = current_y

                image_center = self.cam.w_centre
                pixel_error = blue_dot.cx() - image_center

                error_normalized = pixel_error / (image_center)

                steering = self.PID.get_pid(error_normalized, 1.25)
                print(steering)

                steering = max(min(steering, 1.0), -1.0)

                self.drive(speed, steering)
            else:
                break
        self.servo.set_speed(speed, speed)
        time.sleep(0.6)
        self.servo.set_speed(0,0)
        time.sleep(0.5)
        if green is not None:
            return green
        return None

    '''
    def centering(self, pan_start=0) -> None:

        print("starting centering")
        self.servo.set_angle(pan_start)
        time.sleep(1)
        frames = 0

        while frames < 5:
            # Get blobs
            blobs, _ = self.cam.get_blobs()
            blue_dot = self.cam.find_colour_biggest_blob(blobs, self.blue_dot_id)

            if blue_dot is not None:
                frames = 0
                error, pan_angle = self.track_blob(blue_dot)
                print(f"pan angle {pan_angle}")
                if abs(error) < 0.05:
                    if pan_angle < 4:
                        self.servo.set_angle(0)
                        print("Finished centering")
                        break

                    speed = max(0.08, pan_angle*0.01)
                    self.servo.set_speed(pan_angle*0.01, -pan_angle*0.01)
                    time.sleep(0.05)
                    self.servo.set_speed(0,0)

            else:
                frames += 1
                time.sleep(0.01)
    '''
    def Centering(self, turn_speed: float = 0.08) -> None:
        print("Starting centering function")
        count=0
        while True:
            # Get detected blobs and the image from the camera.
            blobs, img = self.cam.get_blobs()

            # Calculate the camera's horizontal center.
            cam_center = self.cam.w_centre
            if blobs:
                # Select the blob whose center is closest to the camera center.
                selected_blob = min( blobs, key=lambda b: abs(b.cx() - cam_center))
                error = selected_blob.cx() - cam_center
                print("Selected blob (code:", selected_blob.code(), ") error:", error)

                # Define the tolerance region as cam_center ± (img.width()/20)
                tolerance = img.width() / 16.0

                # Rotate in place slowly.
                if abs(error) >= tolerance and error < 0:
                    print("Blob is to the left. Rotating left slowly.")
                    self.servo.set_speed(turn_speed, 0)
                elif abs(error) >= tolerance and error > 0:
                    print("Blob is to the right. Rotating right slowly.")
                    self.servo.set_speed(0, turn_speed)
                else:
                    self.servo.set_speed(0, 0)
                    count+=1
            else:
                count+=1

            if count>5:
                break

            time.sleep_ms(50)  # Short delay to allow sensor update
            self.servo.set_speed(0, 0)
            time.sleep_ms(100)
    

    def check_blocks_ahead(self) -> None:
        """
        Check if there are any blocks ahead of the robot.
        """

        # Check ahead
        self.servo.set_angle(0)
        time.sleep(1)
        if self.blob_colour_ahead() is self.blue_dot_id:
            return 0

        # Check left
        self.servo.set_angle(60)
        time.sleep(1)
        if self.blob_colour_ahead() is self.blue_dot_id:
            self.servo.set_angle(0)
            return 1

        # Check Right
        self.servo.set_angle(-60)
        time.sleep(1)
        if self.blob_colour_ahead() is self.blue_dot_id:
            self.servo.set_angle(0)
            return 2



    """Helper Functions"""

    def blob_colour_ahead(self):
        """
        Check the colour of the biggest blob ahead
        """

        # Get blobs
        blobs, _ = self.cam.get_blobs()
        biggest_blob = self.cam.get_biggest_blob(blobs)

        # Find the biggest blob
        frames = 0
        while frames < 4 and biggest_blob is None:
            blobs, _ = self.cam.get_blobs()
            biggest_blob = self.cam.get_biggest_blob(blobs)
            time.sleep(0.01)
            frames += 1

        if biggest_blob is None:
            return None

        # Check the colour of the biggest blob
        colour = self.cam.find_blob([biggest_blob], 0)

        return colour

    def drive(self, drive: float, steering: float) -> None:
        """
        Differential drive function for the robot.

        Args:
            drive (float): Speed to set the servos to (-1~1)
            steering (float): Sets the steering to (-1~1)
        """
        # Apply limits
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

        # print(f"Horizontal deviation: {angle_horizontal:.2f}°, Vertical angle: {theta_obstacle:.2f}°")
        # print(f"Horizontal distance to obstacle: {distance:.2f} cm")
        # print(f"Total distance to obstacle: {actual_distance:.2f} cm")
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

    # def centering(self, turn_speed=0.07, distance_threshold: float = 30) -> None:
    #     """
    #     Centering function:
    #     This function continuously adjusts the robot's orientation in place by slowly rotating it.
    #     It automatically selects the blob (blue, red, or green) that is closest to the vertical center line
    #     of the camera image. It loops until the selected blob falls within the narrow central region
    #     (defined as cam_center ± (img.width()/20)). Once aligned, it then checks the distance to the target.
    #     If the distance is above the defined threshold, it moves forward slowly until the object is closer.

    #     Parameters:
    #         cam (Cam): The camera object used to capture blobs.
    #         servo (Servo): The servo control object used to drive the robot.
    #         turn_speed (float): The fixed turning speed used for gradual rotation.
    #         distance_threshold (float): The maximum allowed distance (in cm) from the target. If the calculated
    #                                     distance is above this value, the robot moves forward until within threshold.
    #     """
    #     print("Starting centering function")
    #     while True:
    #         clock.tick()
    #         # Get detected blobs and the image from the camera.
    #         blobs, img = self.cam.get_blobs()

    #         print(blobs)

    #         # If no valid blob is detected, stop any motion.
    #         if not blobs:
    #             print("No valid blobs detected. Stopping adjustment.")
    #             self.servo.set_speed(0, 0)
    #             break

    #         # Calculate the camera's horizontal center.
    #         cam_center = self.cam.w_centre

    #         # Select the blob whose center is closest to the camera center.
    #         selected_blob = min(blobs, key=lambda b: abs(b.cx() - cam_center))
    #         error = selected_blob.cx() - cam_center
    #         print("Selected blob (code:", selected_blob.code(), ") error:", error)

    #         # Define the tolerance region as cam_center ± (img.width()/20)
    #         tolerance = img.width() / 20.0

    #         # Check if the blob is already within the center region.
    #         if abs(error) <= tolerance:
    #             print("Blob is within the central region. Alignment achieved.")
    #             self.servo.set_speed(0, 0)
    #             # Now check the distance using calculate_distance.
    #             distance, angle_horizontal = self.calculate_distance(selected_blob)
    #             print("Calculated distance to object: {:.2f} cm".format(distance))
    #             if distance <= distance_threshold:
    #                 print("Distance is within threshold. No forward movement required.")
    #                 break
    #             else:
    #                 print("Distance is above threshold. Moving forward until within threshold.")
    #                 # Move forward until the distance becomes less than or equal to the threshold.
    #                 while True:
    #                     self.servo.set_speed(turn_speed, turn_speed)
    #                     time.sleep(0.1)
    #                     # Re-capture the image and update distance.
    #                     blobs, img = self.cam.get_blobs()
    #                     if not blobs:
    #                         print("No valid blob found during forward movement. Stopping.")
    #                         self.servo.set_speed(0, 0)
    #                         break
    #                     cam_center = self.cam.w_centre
    #                     selected_blob = min(blobs, key=lambda b: abs(b.cx() - cam_center))
    #                     error = selected_blob.cx() - cam_center
    #                     distance, angle_horizontal = self.calculate_distance(selected_blob)
    #                     print("Updated distance: {:.2f} cm".format(distance))
    #                     if distance <= distance_threshold:
    #                         print("Reached target distance threshold.")
    #                         self.servo.set_speed(0, 0)
    #                         break
    #                 break
    #         else:
    #             # Rotate in place slowly.
    #             if error < 0:
    #                 print("Blob is to the left. Rotating left slowly.")
    #                 self.servo.set_speed(turn_speed, 0)
    #             else:
    #                 print("Blob is to the right. Rotating right slowly.")
    #                 self.servo.set_speed(0, turn_speed)
    #         if abs(error) <= tolerance and distance <= distance_threshold:
    #             break
    #         time.sleep(0.1)  # Short delay to allow sensor update


    def pan(self):
        """
        Rotate the servo to multiple angles and perform sub-pan scanning at each angle.
        Determine the final output angle and whether the final line is detected.
        """
        # Initialize default variables:
        # angle_to_output: None indicates no target has been found yet.
        # boundary: Indicates detected boundary ('L' or 'R').
        # obstacle: Flag for obstacle detection.
        # final_line: Flag for final line detection.
        angle_to_output = None
        boundary = None
        obstacle = False
        final_line = False

        # Set servo to 0 degrees and perform a sub-pan scan
        self.servo.set_angle(0)
        angle_to_output, boundary, obstacle = self.sub_pan(angle_to_output, boundary, obstacle, 0)
        print(angle_to_output)
        time.sleep(0.1)

        # Set servo to 45 degrees and perform a sub-pan scan
        self.servo.set_angle(65)
        time.sleep(0.5)
        angle_to_output, boundary, obstacle = self.sub_pan(angle_to_output, boundary, obstacle, 45)
        print(angle_to_output)
        time.sleep(0.1)

        # Set servo to -45 degrees and perform a sub-pan scan
        self.servo.set_angle(-65)
        time.sleep(0.5)
        angle_to_output, boundary, obstacle = self.sub_pan(angle_to_output, boundary, obstacle, -45)
        print(angle_to_output)
        time.sleep(0.1)

        # Determine final output based on scan results:
        if angle_to_output is None and not obstacle:
            final_line = True
        elif angle_to_output is None and obstacle:
            if boundary == 'L':
                angle_to_output = -90
            elif boundary == 'R':
                angle_to_output = 90

        print(angle_to_output)
        # Reset servo to 0 degrees
        self.servo.set_angle(0)

        return angle_to_output, final_line
    
    def check_and_move_green(self, speed, distance_threshold=50):
        """
        Follow the blue dot on the screen.
        """
        old_y = None

        while True:
            # Get blobs

            frames = 0
            blobs, _ = self.cam.get_blobs()
            green_dot = self.cam.find_colour_biggest_blob(blobs, self.green_goal)
            while frames < 10 and green_dot is None:
                self.servo.set_speed(0,0)
                blobs, _ = self.cam.get_blobs()

                green_dot = self.cam.find_colour_biggest_blob(blobs, self.green_goal)
                time.sleep(0.02)
                frames += 1

            if green_dot is not None:

                if old_y is None:
                    old_y = green_dot.cy()

                current_y = green_dot.cy()

                if current_y - old_y < -distance_threshold:
                    break

                old_y = current_y

                image_center = self.cam.w_centre
                pixel_error = green_dot.cx() - image_center

                error_normalized = pixel_error / (image_center)

                steering = self.PID.get_pid(error_normalized, 1.25)

                steering = max(min(steering, 1.0), -1.0)

                self.drive(speed, steering)
            else:
                return False
        self.servo.set_speed(speed, speed)
        time.sleep(0.6)
        self.servo.set_speed(0,0)
        time.sleep(0.5)
        return True




    def sub_pan(self, angle_to_output, boundary, obstacle, turn_angle):
        """
        Perform a scanning routine to detect targets and obstacles, and update the output angle,
        boundary, and obstacle status accordingly.

        Parameters:
            angle_to_output: Current output angle (None if no target is found yet).
            boundary: Current boundary value.
            obstacle: Boolean flag indicating if an obstacle is detected.
            turn_angle: The servo angle for the current scan.

        Returns:
            Updated angle_to_output, boundary, and obstacle status.
        """
        count = 0
        green_target_index=None
        blue_target_index=None
        obstacle_index=None
        while True:
            # Get blobs and image from the camera
            blobs, img = self.cam.get_blobs_upper()
            if blobs:
                #print('ls: blobs found!')
                green_target_index = self.cam.find_blob(blobs, 2)
                blue_target_index = self.cam.find_blob(blobs, 0)
                #you may comment those 2 lines below if you haven't record the threshold of boundry
                #boundary_index_L = self.cam.find_blob(blobs, self.boundry_L_id)
                #boundary_index_R = self.cam.find_blob(blobs, self.boundry_R_id)
                obstacle_index = self.cam.find_blob(blobs, 1)

                # If the green target is detected, return the current turn angle
                if green_target_index is not None:
                    print('green')
                    return turn_angle, boundary, obstacle

                # If the blue target is detected, choose the turn angle if no target has been set
                if blue_target_index is not None:
                    #Determine whether the angle_to_output has been changed by the previous sub_pan
                    if angle_to_output is None: #this means it was not changed
                        print('blue')
                        return turn_angle, boundary, obstacle

                    else:#this means it was changed by previous sub_pan. so we will not change the angle_to_output although we detect the blue bolbs
                        return angle_to_output, boundary, obstacle

                # Update obstacle status if an obstacle is detected (for final line detection)
                if obstacle_index is not None:
                    obstacle = True

                '''
                # Update boundary based on left/right detection
                if boundary_index_L:
                    boundary = 'L'
                elif boundary_index_R:
                    boundary = 'R'
                '''

            count += 1
            time.sleep(0.05)  # Pause briefly before the next iteration
            if count > 10:
                # Return current state if no target is detected after several iterations
                return angle_to_output, boundary, obstacle

    def turn(self, angle):
        if angle>=0:
            self.servo.turn_R(abs(angle))
        elif angle<=0:
            self.servo.turn_L(abs(angle))


    def redo_pan(self):
        angle_to_output = None
        final_line = False

        self.turn(-10)
        angle_to_output,final_line=self.pan()
        if angle_to_output is not None:
            return angle_to_output,final_line

        self.turn(20)
        angle_to_output,final_line=self.pan()
        if angle_to_output is not None:
            return angle_to_output,final_line

        self.turn(-10)

        self.servo.set_speed(-0.1, -0.1)
        time.sleep(0.2)
        self.servo.set_speed(0,0)
        angle_to_output,final_line=self.pan()
        if angle_to_output is not None:
            return angle_to_output,final_line

        self.servo.set_speed(0.1, 0.1)
        time.sleep(0.2)
        self.servo.set_speed(0,0)
        angle_to_output,final_line=self.pan()
        if angle_to_output is not None:
            return angle_to_output,final_line

        return angle_to_output,final_line

