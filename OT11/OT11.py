"""OT11 - World model."""
import PiBot
import math


class Robot:
    """The robot class."""

    def __init__(self, initial_odometry=[0, 0, 0]):
        """
        Initialize variables.

        Arguments:
          initial_odometry -- Initial odometry(start position and angle),
                              [x, y, yaw] in [meters, meters, radians]
        """
        self.robot = PiBot.PiBot()
        self.delta_time = 0
        self.left_delta = 0
        self.right_delta = 0
        self.left_wheel_speed = 0
        self.right_wheel_speed = 0
        self.encoder_odometry = initial_odometry.copy()
        self.wheel_radius = 0

        self.time = 0
        self.last_time = 0

        self.camera_resolution = self.robot.CAMERA_RESOLUTION[0]
        self.camera_y_resolution = self.robot.CAMERA_RESOLUTION[1]
        self.camera_field_of_view = self.robot.CAMERA_FIELD_OF_VIEW[0]
        self.camera_center = self.camera_resolution / 2

        self.left_encoder = 0
        self.last_left_encoder = 0
        self.right_encoder = 0
        self.last_right_encoder = 0

        self.detected_objects = []
        self.closest_object_angle = None
        self.closest_object = None
        self.last_object_id = 1
        self.world_objects = {}

        self.closest_object = 0

        self.coordinates_xy = ()
        self.camera_resolution = self.robot.CAMERA_RESOLUTION[0]
        self.camera_field_of_view = self.robot.CAMERA_FIELD_OF_VIEW[0]
        self.camera_center = self.camera_resolution / 2


    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the API reference."""
        self.robot = robot

    def get_encoder_odometry(self):
        """
        Return the encoder odometry.

        Returns:
           A tuple with x, y coordinates and yaw angle (x, y, yaw)
           based on encoder data. The units must be (meters, meters, radians).
        """
        return self.encoder_odometry[0], self.encoder_odometry[1], self.encoder_odometry[2]

    def update_world(self) -> None:
        """Update the world model (insert objects into memory)."""
        if len(self.detected_objects) == 0:
            pass
        for object in self.detected_objects:
            object_x = object[1][0]
            object_distance = 30 / object[2]
            object_x_difference = self.camera_center - object_x
            object_angle = math.radians((object_x_difference / self.camera_resolution) * self.camera_field_of_view)
            object_world_angle = object_angle + self.encoder_odometry[2]
            object_x = self.encoder_odometry[0] + object_distance * math.cos(object_world_angle)
            object_y = self.encoder_odometry[1] + object_distance * math.sin(object_world_angle)
            self.world_objects[self.last_object_id] = (object_x, object_y)
            self.last_object_id += 1

    def get_closest_object_angle(self) -> float:
        """
        Return the angle of the closest object.

        This method returns the angle of the object that is
        the shortest Euclidean distance
        (i.e., the closest object angle w.r.t. the robot heading).

        Returns:
          The normalized (range [0..2*pi]) angle (radians) to the
          closest object w.r.t. the robot heading following
          the right-hand rule.
          E.g., 0 if object is straight ahead,
                1.57 if the  object is 90 degrees to the left of the robot.
                3.14 if the closest object is 180 degrees from the robot.
                4.71 if the objectis 90 degrees to the right of the robot.
          None if no objects have been detected.
        """
        self.closest_object = 0
        closest_distance = 99999
        for object in range(len(self.world_objects)):
            x = self.world_objects[object][0]
            y = self.world_objects[object][1]
            object_distance = math.sqrt(((x - self.encoder_odometry[0]) ** 2) + ((y - self.encoder_odometry[1]) ** 2))
            if object_distance < closest_distance:
                closest_distance = object_distance
                self.closest_object = object
        if self.closest_object > 0:
            angle = math.atan2(self.world_objects[self.closest_object][1] - self.encoder_odometry[1], self.world_objects[self.closest_object][0] - self.encoder_odometry[0]) - self.encoder_odometry[2]
            return angle
        else:
            return None

    def sense(self):
        """SPA architecture sense block."""
        # Read the current time
        self.last_time = self.time
        self.time = self.robot.get_time()
        self.delta_time = self.time - self.last_time

        # Read wheel encoder values
        self.last_left_encoder = self.left_encoder
        self.left_encoder = self.robot.get_left_wheel_encoder()
        self.last_right_encoder = self.right_encoder
        self.right_encoder = self.robot.get_right_wheel_encoder()
        self.left_delta = self.left_encoder - self.last_left_encoder
        self.right_delta = self.right_encoder - self.last_right_encoder

        self.detected_objects = self.robot.get_camera_objects()

        #odometry
        if self.delta_time > 0:
            self.left_wheel_speed = math.radians(self.left_delta) / self.delta_time
            self.right_wheel_speed = math.radians(self.right_delta) / self.delta_time
            self.encoder_odometry[2] = (self.wheel_radius / self.robot.AXIS_LENGTH) * (
                        math.radians(self.right_encoder) - math.radians(self.left_encoder))
            self.encoder_odometry[0] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.cos(
                self.encoder_odometry[2]) * self.delta_time
            self.encoder_odometry[1] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.sin(
                self.encoder_odometry[2]) * self.delta_time

    def spin(self):
        """Spin loop."""
        for _ in range(100):
            self.sense()
            print(f"objects = {self.robot.get_camera_objects()}")
            self.robot.sleep(0.05)

#
# def main():
#     """The main entry point."""
#     robot = Robot()
#     robot.spin()
#
#
# if __name__ == "__main__":
#     main()
def test():
    robot = Robot([0.201, -0.148, 1.529])  # initial odometry values (to compare with ground truth)
    import turn_and_straight
    data = turn_and_straight.get_data()
    robot.robot.load_data_profile(data)

    last_update = robot.robot.get_time()
    for _ in range(int(robot.robot.data[-1][0]/0.05)):
        robot.sense()
        print(f"ground truth = {robot.robot.get_ground_truth()}")
        if last_update + 1 < robot.robot.get_time():
            robot.update_world()
            last_update = robot.robot.get_time()
        robot.robot.sleep(0.05)
    print(f"closest object angle = {robot.get_closest_object_angle()}")


if __name__ == "__main__":
    test()
