"""OT10 - Robot vision processing."""
import math

import PiBot


class Robot:
    """Robot class."""

    def __init__(self):
        """Initialize variables."""
        self.robot = PiBot.PiBot()
        self.detected_objects = []

        self.camera_resolution = self.robot.CAMERA_RESOLUTION[0]
        self.camera_field_of_view = self.robot.CAMERA_FIELD_OF_VIEW[0]
        self.camera_center = self.camera_resolution / 2

        self.coordinates_xy = ()
        self.closest_object_angle = None
        self.closest_object = None

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the API reference."""
        self.robot = robot

    def get_closest_visible_object_angle(self):
        """
        Find the closest visible object from the objects list.

        Returns:
          The angle (in radians) to the closest object w.r.t. the robot
          orientation (i.e., 0 is directly ahead) following the right
          hand rule (i.e., objects to the left have a plus sign and
          objects to the right have a minus sign).
          Must return None if no objects are visible.
        """
        return self.closest_object_angle

    def sense(self):
        """SPA architecture sense block."""
        self.detected_objects = self.robot.get_camera_objects()
        self.cameradetection()

    def cameradetection(self):
        """Calculating the closest object angle"""
        if len(self.detected_objects) == 0:
            return None
        else:
            if self.detected_objects[0][2] > self.detected_objects[1][2]:
                self.coordinates_xy = self.detected_objects[0][1]
                self.closest_object = self.detected_objects[0][0]
            if self.detected_objects[0][2] < self.detected_objects[1][2]:
                self.coordinates_xy = self.detected_objects[1][1]
                self.closest_object = self.detected_objects[1][0]
            else:
                self.coordinates_xy = self.detected_objects[0][1]
                self.closest_object = self.detected_objects[0][0]

            coordinates_x = self.coordinates_xy[0]
            x_difference = self.camera_center - coordinates_x
            closest_object_angle = (x_difference / self.camera_resolution) * self.camera_field_of_view
            closest_object_angle = round((closest_object_angle * math.pi) / 180, 2)
            self.closest_object_angle = closest_object_angle
            return closest_object_angle

    def spin(self):
        """The spin loop."""
        for _ in range(100):
            self.sense()
            self.robot.sleep(0.05)


def main():
    """Main entry point."""
    robot = Robot()
    robot.spin()

#
# def test():
#     robot = Robot()
#     import blue_approach  # or any other data file
#     data = blue_approach.get_data()
#     robot.robot.load_data_profile(data)
#     for i in range(len(data)):
#         robot.spin()
#         print(f"objects = {robot.robot.get_camera_objects()}")
#         print(robot.camera_center)
#         print(robot.camera_resolution, robot.camera_field_of_view)
#         print(robot.closest_object_angle)
#         robot.robot.sleep(0.05)


if __name__ == "__main__":
    main()
