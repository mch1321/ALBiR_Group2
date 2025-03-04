
    def stage1(self, speed: float) -> None:
        """
        Line following algorithm

        Args:
            speed (float): Speed to set the servos to (-1~1)
            bias (float): Just an example of other arguments you can add to the function!
        """
        print("Running Stage 1")
        i = 0
        while True:
            # Get blobs from the camera
            blobs, img = self.cam.get_blobs()

            # Initialize steering variables
            found_mid, found_l, found_r = None, None, None
            pixel_error = 0
            # error_angle = 0

            if blobs:
                biggest_blob = self.cam.get_biggest_blob(blobs)

                # Update pan and get error values
                # error_angle, pan_angle = self.tuning.update_pan(biggest_blob)

                # Find lane-related blobs
                found_mid_index = self.cam.find_blob(blobs, self.mid_line_id)
                found_l_index = self.cam.find_blob(blobs, self.l_line_id)
                found_r_index = self.cam.find_blob(blobs, self.r_line_id)

                if found_mid_index is not None:
                    found_mid = blobs[found_mid_index]
                if found_l_index is not None:
                    found_l = blobs[found_l_index]
                if found_r_index is not None:
                    found_r = blobs[found_r_index]


            # Steering logic
            if found_mid:
                print("Found Middle")
                image_center = self.cam.w_centre
                pixel_error = found_mid.cx() - image_center
                print("Pixel Error", pixel_error)

                # Normalize error to [-1, 1]
                error_normalized = pixel_error / image_center

                # Update PID with the new error
                steering = self.PID.get_pid(error_normalized, 1)

                # Clip steering to [-1,1]
                steering = max(min(steering, 1.0), -1.0)

                # Drive the robot using the computed steering
                self.drive(speed, steering)

            elif found_l and found_r:
                print("Found Left and Right")
                image_center = img.width() / 2.0
                lane_center = (found_l.cx() + found_r.cx()) / 2.0
                pixel_error = lane_center - image_center

                error_normalized = pixel_error / (img.width() / 2.0)
                steering = self.PID.get_pid(error_normalized, 1)
                steering = max(min(steering, 1.0), -1.0)
                self.drive(speed, steering)

            else:
                print("Nothing Found")
                self.drive(0.0, 0.0)

                while i < 30:
                    print("Executing loop")
                    i += 5
                    self.servo.set_angle(i)
                    time.sleep(0.1)
                    print("Set Angle:", i)

                    blobs, _ = self.cam.get_blobs()
                    if blobs:
                        # self.servo.set_angle(0)
                        break
