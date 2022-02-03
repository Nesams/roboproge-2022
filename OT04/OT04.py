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
        self.line = [self.leftmost, self.second_from_left, self.third_from_right, self.third_from_left, self.second_from_right, self.rightmost]

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