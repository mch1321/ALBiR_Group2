from servos import Servo
from camera import Cam
from pid_control import PID
import sensor
import os, time

class PanTuning(object):
    """
    A class for managing PID tuning, servo calibration, and camera adjustments.
    """
    def __init__(self, thresholds, gain = 5, p=0.22, i=0.0, d=0.0, imax=0.0):
        """
        Initialise the Tuning object with given PID parameters.

        Args:
            thresholds (list): Colour detection thresholds
            gain (float): Camera gain
            p (float): Proportional gain.
            i (float): Integral gain.
            d (float): Derivative gain.
            imax (float): Maximum Integral error.
        """
        self.servo = Servo()
        self.servo.soft_reset()
        self.cam = Cam(thresholds, gain)
        self.PID = PID(p, i, d, imax)

        self.min_angle = 0
        self.max_angle = 0
        self.targetmax_angle = 10
        self.targetmin_angle = -self.targetmax_angle

        self.csv = None

    def measure(self, freq):
        """
        Measures the tracking error and pan angle of the
        red square target for a specified frequency of oscillation.

        Args:
            freq (int): Frequency of oscillation in (Hz).
        """
        # Track 10 periods of oscillations
        t_run = 1000*10/freq

        self.prepare_csv(freq)

        # Set up flag for searching for target
        flag = True

        # Check if calibration has been done
        self.calibrate()
        while ((self.min_angle > self.targetmin_angle) or
               (self.max_angle < self.targetmax_angle)):
            print('Calibration failed')
            print('Make sure you have calibrated thresholds')
            print('If you have, please put the robot closer to the screen!')
            self.calibrate()
        print('Calibration complete')
        # reset pan to max angle
        self.servo.set_angle(0)

        while flag is True:
            # Get list of blobs and biggest blob
            blobs, img = self.cam.get_blobs()
            big_blob = self.cam.get_biggest_blob(blobs)

            # Check biggest blob is not None and is red for target then pass
            if big_blob and self.cam.find_blob([big_blob], 0) is not None:
                flag = False

        # Setup times for freq test
        t_start = time.ticks_ms()
        t_end =  time.ticks_add(t_start, int(t_run))
        print(t_end)
        print("step1")
        count = 0
        while time.ticks_diff(t_end, time.ticks_ms()) > 0:
            # Get new image and blocks
            # Get list of blobs and biggest blob
            blobs, img = self.cam.get_blobs()
            big_blob = self.cam.get_biggest_blob(blobs)
            print("find")

            if big_blob and self.cam.find_blob([big_blob], 0) is not None:
                error, target_angle = self.update_pan(big_blob)
                # Write data to csv
                run_time = time.ticks_diff(time.ticks_ms(), t_start)
                self.write_csv([run_time, error, target_angle])
                print("write")

        # Close and exit
        self.close_csv()
        print("csv closed")

    def calibrate(self):
        """
        Calibrate the pan positioning by setting max and min angles.
        Provides feedback during the process.
        """
        print('Please start the target tracking video')
        self.max_angle = 0
        self.min_angle = 0
        self.servo.set_angle(0)

        #  Set up clock for FPS and time tracking
        t_lost = time.ticks_add(time.ticks_ms(), 3000)

        # Loop until target is lost
        while time.ticks_diff(t_lost, time.ticks_ms()) > 0:

            # Get list of blobs and biggest blob
            blobs, img = self.cam.get_blobs()
            big_blob = self.cam.get_biggest_blob(blobs)
            # Check biggest blob is not None and is blue for calibration
            if big_blob and self.cam.find_blob([big_blob], 1) is not None:
                # track the calibration target
                error, pan_angle = self.update_pan(big_blob)

                # Update tuning curve parameters
                if error < 10:
                    if pan_angle < self.min_angle:
                        self.min_angle = pan_angle
                        print('New min angle: ', self.min_angle)
                    if pan_angle > self.max_angle:
                        self.max_angle = pan_angle
                        print('New max angle: ', self.max_angle)

                # As block was found reset lost timer
                t_lost = time.ticks_add(time.ticks_ms(), 1500)


    def update_pan(self, blob) -> tuple:
        """
        Adjust the camera pan by changing the servo angle based on the given blob's position.

        Args:
            blob (blob): Object retrieved from find_blobs - see OpenMV docs

        Returns:
            angle_error (float): The difference in angle between the blob and pan servo
            pan_angle (float): Angle of the pan wrt. heading
        """
        # Error between camera angle and target in pixels
        pixel_error = blob.cx() - self.cam.w_centre

        # Convert error to angle
        angle_error = -(pixel_error/sensor.width()*self.cam.h_fov)

        pid_error = self.PID.get_pid(angle_error,1)

        # Error between camera angle and target in ([deg])
        pan_angle = self.servo.pan_pos + pid_error

        # Move pan servo to track block
        self.servo.set_angle(pan_angle)

        return angle_error, pan_angle


    def prepare_csv(self, freq: int) -> str:
        """
        Prepare CSV file for tracking data.

        Args:
            freq (int): Frequency (Hz) for naming the file.
        """
        # Set file ext counter to 0
        file_n = 0

        # Try making ./CSV directory if it doesn't exist
        try:
            os.mkdir("./CSV")
        except OSError as e:
            pass

        # Generate initial file name
        filename = "./CSV/Curve" + str(freq) + "Hz_" + str(file_n) + ".csv"

        # Check if file already exists and increment counter if it does
        while True:
            try:
                with open(filename, 'r'):
                    pass
                # If file exists, increment the counter and try again
                file_n += 1
                filename = "./CSV/Curve" + str(freq) + "Hz_" + str(file_n) + ".csv"
            except OSError:
                # If file doesn't exist, break out of loop
                break

        print("Opening:", filename)

        self.csv = open(filename, 'w')
        self.csv.flush()
        self.csv.write('Time,Error,Angle\n')  # Writing headers


    def write_csv(self, data: tuple) -> None:
        """
        Write tracking data to a CSV file.

        Args:
            data (tuple): Tuple containing lists of data to write to CSV file.
        """
        self.csv.write(','.join(map(str, data)) + '\n')


    def close_csv(self) -> None:
        """
        Close the CSV file.
        """
        try:
            self.csv.flush()
            self.csv.close()
        except:
            pass
        print('File Closed')
