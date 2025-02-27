import sensor, time

class Cam(object):
    """
    The Cam class manages the camera sensor for image capturing, processing,
    and color tracking. It initializes the camera parameters and sets the color
    thresholds for blob detection.
    """

    def __init__(self, thresholds, gain = 25):
        """
        Initialise the Cam object by setting up camera parameters and
        configuring color thresholds.
        """
        # Configure camera settings
        sensor.reset()
        sensor.set_pixformat(sensor.RGB565)
        sensor.set_framesize(sensor.VGA)   # Set frame size to 640x480
        sensor.skip_frames(time=2000)   # Allow the camera to adjust to light levels

        # Both must be turned off for color tracking
        sensor.set_auto_gain(False, gain_db = gain)
        sensor.set_auto_whitebal(False)

        # Initialise sensor properties
        self.w_centre = sensor.width()/2
        self.h_centre = sensor.height()/2
        self.h_fov = 31.5
        self.v_fov = 21
        self.camera_elevation_angle = -11.5  # can measure and adjust this value
        self.clock = time.clock()

        # Define color tracking thresholds for Red, Green, Blue, and Yellow colors
        # Thresholds are in the order of (L Min, L Max, A Min, A Max, B Min, B Max)
        self.thresholds = thresholds


    def get_blobs(self, angle = 0) -> tuple:
        """
        Capture an image and detect color blobs based on predefined thresholds.
        Args:
            angle (float): pan angle for image rotation correction. Defaults to 0.
        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()
        img.rotation_corr(z_rotation=angle)
        blobs = img.find_blobs(self.thresholds,pixels_threshold=60,area_threshold=60)

        return blobs, img


    def get_blobs_bottom(self, angle = 0) -> tuple:
        """
        Capture an image and detect colour blobs based on predefined thresholds.
        Region of interest is set to the bottom 2/3 of the image.
        Args:
            angle (float): pan angle for image rotation correction. Defaults to 0.
        Returns:
            blobs (list): List of detected blobs.
            img (image): Captured image used to find blobs.
        """
        img = sensor.snapshot()
        img.rotation_corr(z_rotation=angle)

        blobs = img.find_blobs(self.thresholds,pixels_threshold=150,area_threshold=150,
                               roi=(1,int(sensor.height()/3),
                                    int(sensor.width()),int(2*sensor.height()/3)))

        return blobs, img


    def get_biggest_blob(self, blobs):
        """
        Identify and return the largest blob from a list of detected blobs.

        Args:
            blobs (list): List of detected blobs.

        Returns:
            big_blob (blob): The biggest blob from list - see OpenMV docs for blob class.
        """
        max_pixels = 0
        big_blob = None

        for blob in blobs:
            # Update the big blob if the current blob has more pixels
            if blob.pixels() > max_pixels:
                max_pixels = blob.pixels()
                big_blob = blob

        return big_blob


    def get_blob_colours(self, blobs) -> int:
        """
        Returns the binary code (as int) of thresholds met by each blob

        Args:
            blobs (list): List of detected blobs.

        Returns:
            colours (list): List of binary codes (as int) of thresholds met by each element in blobs.
        """
        colours = []

        for blob in blobs:
            colours.append(blob[8])

        return colours


    def find_blob(self, blobs, threshold_idx: int):
        """
        Finds the first blob in blobs that was detected using a specified threshold

        Args:
            blobs (list): List of detected blobs.
            threshold_idx (int): Index along self.thresholds.

        Returns:
            found_idx (int): Index along blobs for the first blob that was detected using self.thresholds(threshold_idx)
        """
        colours = self.get_blob_colours(blobs)

        for found_idx, colour in enumerate(colours):
            if colour == pow(2, threshold_idx):
                return found_idx

        return None


def calculate_distance(obstacle_blob, camera):
    """
    Calculate the distance from the robot to the detected obstacle, using the bottom center of the bounding box.
    """
    h_cam = 6  # Camera height from the ground in cm
    theta_cam = camera.camera_elevation_angle  # Camera downward tilt angle in degrees
    v_fov, h_fov = camera.v_fov, camera.h_fov  # Camera field of view angles in degrees
    img_width, img_height = sensor.width(), sensor.height()

    # Use bottom center of the bounding box
    bottom_center_x = obstacle_blob.cx()
    bottom_center_y = obstacle_blob.y() + obstacle_blob.h()

    # Calculate vertical angle deviation
    pixel_error_y = camera.h_centre - bottom_center_y
    angle_y = (pixel_error_y * v_fov) / img_height
    theta_obstacle = -theta_cam + angle_y

    # Calculate horizontal angle deviation
    pixel_error_x = bottom_center_x - camera.w_centre
    angle_horizontal = (pixel_error_x * h_fov) / img_width

    # Compute horizontal distance
    distance = h_cam * math.tan(math.radians(theta_obstacle)) if theta_obstacle > 0 else float('inf')
    actual_distance = distance / math.cos(math.radians(abs(angle_horizontal)))  # Adjusted for horizontal deviation

    print(f"Horizontal deviation: {angle_horizontal:.2f}°, Vertical angle: {theta_obstacle:.2f}°")
    print(f"Horizontal distance to obstacle: {distance:.2f} cm")
    print(f"Total distance to obstacle: {actual_distance:.2f} cm")
    return actual_distance, angle_horizontal


if __name__ == "__main__":
    import math
    thresholds = [
        (53, 71, -25, -4, 0, 32)  # Green detection threshold
    ]

    camera = Cam(thresholds,10)
    camera.camera_elevation_angle=-68
    while True:
        blobs, img = camera.get_blobs()
        obstacle_blob = camera.get_biggest_blob(blobs)

        if obstacle_blob:
            img.draw_rectangle(obstacle_blob.rect())
            img.draw_cross(obstacle_blob.cx(), obstacle_blob.cy())
            distance, angle_horizontal = calculate_distance(obstacle_blob, camera)
        else:
            print("No obstacle detected...")

        time.sleep(0.1)
