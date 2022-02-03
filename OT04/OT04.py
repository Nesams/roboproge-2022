"""OT04 - Line tracking."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class initialization."""
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.leftmost = 0
        self.second_from_left = 0
        self.third_from_left = 0
        self.third_from_right = 0
        self.second_from_right = 0
        self.rightmost = 0
        self.line_directions = [self.leftmost, self.second_from_left, self.third_from_right, self.third_from_left, self.second_from_right, self.rightmost]
        self.last_position = None

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set robot reference."""
        self.robot = robot

    def sense(self):
        """Sense method as per SPA architecture."""
        self.leftmost = self.robot.get_leftmost_line_sensor()
        self.second_from_left = self.robot.get_second_line_sensor_from_left()
        self.third_from_left = self.robot.get_third_line_sensor_from_left()
        self.third_from_right = self.robot.get_third_line_sensor_from_right()
        self.second_from_right = self.robot.get_second_line_sensor_from_right()
        self.rightmost = self.robot.get_rightmost_line_sensor()

    def get_line_direction(self):
        """
        Return the direction of the line based on sensor readings.

        Returns:
          -1: Line is on the right (i.e., the robot should turn right to reach the line again)
           0: Robot is on the line (i.e., the robot should not turn to stay on the line) or no sensor info
           1: Line is on the left (i.e., the robot should turn left to reach the line again)
        """
        left_part = sum(self.line_directions[0:2])
        mid_part = sum(self.line_directions[3:5])
        right_part = sum(self.line_directions[4:])
        if left_part < mid_part and left_part < right_part:
            self.last_position = 1
            return 1
        if right_part < mid_part and right_part < left_part:
            self.last_position = -1
            return -1
        if mid_part < right_part and mid_part < left_part:
            self.last_position = 0
            return 0
        else:
            return self.last_position



    def spin(self):
        """The main spin loop."""
        while not self.shutdown:
            timestamp = self.robot.get_time()
            print(f'timestamp is {timestamp}')
            self.robot.sleep(0.05)
            if self.robot.get_time() > 20:
                self.shutdown = True


def main():
    """Main entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()