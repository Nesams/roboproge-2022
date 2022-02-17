"""OT06 - Object Detection."""
import math
import statistics
import PiBot


class Robot:
    """Robot class."""

    def __init__(self):
        """Class constructor."""
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.objects = []
        self.front_middle_laser = None
        self.filter_list = []
        self.front_middle_laser = 0
        self.left_encoder = 0
        self.right_encoder = 0
        self.wheel_diameter = self.robot.WHEEL_DIAMETER
        self.axis_length = self.robot.AXIS_LENGTH
        self.wheel_circumference = self.robot.WHEEL_DIAMETER * math.pi
        self.object_start = 5
        self.object_start_and_end = []
        self.circle = self.axis_length * math.pi
        self.last_laser_reading = 0.5
        self.object_detected = False
        self.wheel_distance = 0

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set Robot reference."""
        self.robot = robot

    def get_objects(self) -> list:
        """
        Return the list with the detected objects so far.

        (i.e., add new objects to the list as you detect them).

        Returns:
          The list with detected object angles, the angles are in
          degrees [0..360), 0 degrees being the start angle and following
          the right-hand rule (e.g., turning left 90 degrees is 90, turning
          right 90 degrees is 270 degrees).
        """
        if len(self.filter_list) == 5:
            if self.get_front_middle_laser() <= 0.45:
                self.object_detected = True
                self.object_start_and_end.append(self.get_current_angle())
            elif self.get_front_middle_laser() >= 0.5 and self.object_detected:
                object_angle = (self.object_start_and_end[0] + self.object_start_and_end[-1]) / 2
                self.object_detected = False
                self.objects.append(object_angle)
                if 310 <= self.object_start_and_end[0] <= 360 and 0 <= self.object_start_and_end[-1]:
                    self.objects[-1] += 180
                self.object_start_and_end.clear()
        return self.objects

    def get_current_angle(self):
        """"jshdhbd."""
        self.wheel_distance = self.right_encoder / 360 * self.wheel_circumference
        current_angle = self.wheel_distance * 360 / self.circle
        return current_angle % 360

    def get_front_middle_laser(self):
        """
        Return the filtered value.

        Returns:
          None if filter is empty, filtered value otherwise.
        """
        if not self.filter_list:
            return None
        else:
            return statistics.median(self.filter_list)

    def sense(self):
        """Sense method according to the SPA architecture."""
        self.right_encoder = self.robot.get_right_wheel_encoder()
        self.left_encoder = self.robot.get_left_wheel_encoder()
        self.front_middle_laser = self.robot.get_front_middle_laser()
        if self.front_middle_laser is not None:
            self.filter_list.insert(0, self.front_middle_laser)
            self.filter_list = self.filter_list[:5]

    def spin(self):
        """The main loop."""
        while not self.shutdown:
            self.sense()
            self.get_objects()
            print(f'Value is {self.get_front_middle_laser()}')
            self.robot.sleep(0.05)
            if self.robot.get_time() > 20:
                self.shutdown = True


def main():
    """The  main entry point."""
    robot = Robot()
    robot.spin()


def test():
    robot = Robot()
    import close  # or any other data file
    data = close.get_data()
    robot.robot.load_data_profile(data)
    for i in range(len(data)):
        robot.sense()
        robot.get_objects()
        print(f"laser = {robot.robot.get_front_middle_laser()}")
        print(robot.objects)
        robot.robot.sleep(0.05)


if __name__ == "__main__":
    test()
