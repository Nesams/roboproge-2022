"""OT04 - Line tracking."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class initialization."""
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.line_directions = []
        self.last_position = 0

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set robot reference."""
        self.robot = robot

    def sense(self):
        """Sense method as per SPA architecture."""
        self.line_directions = [self.robot.get_leftmost_line_sensor(), self.robot.get_second_line_sensor_from_left(),
                                self.robot.get_third_line_sensor_from_left(), self.robot.get_third_line_sensor_from_right(),
                                self.robot.get_second_line_sensor_from_right(), self.robot.get_rightmost_line_sensor()]

    def get_line_direction(self):
        """
        Return the direction of the line based on sensor readings.

        Returns:
          -1: Line is on the right (i.e., the robot should turn right to reach the line again)
           0: Robot is on the line (i.e., the robot should not turn to stay on the line) or no sensor info
           1: Line is on the left (i.e., the robot should turn left to reach the line again)
        """
        line_exists = []
        for sensor in self.line_directions:
            if sensor <= 400:
                line_exists.append(1)
            else:
                line_exists.append(0)

        if line_exists[0] == 1 or line_exists[1] == 1:
            self.last_position = 1
            return 1
        if line_exists[2] == 1 or line_exists[3] == 1:
            self.last_position = 0
            return 0
        if line_exists[4] == 1 or line_exists[5] == 1:
            self.last_position = -1
            return -1
        else:
            return self.last_position
        #  if line_exists[0:3].count(1) > line_exists[3:].count(1) or line_exists[2] == 1:
        #     self.last_position = 1
        #     return 1
        # if line_exists[0:3].count(1) < line_exists[3:].count(1) or line_exists[3] == 1:
        #     self.last_position = -1
        #     return -1
        # if line_exists.count(1) == 0:
        #     return self.last_position
        # else:
        #     self.last_position = 0
        #     return 0


    def spin(self):
        """The main spin loop."""
        while not self.shutdown:
            timestamp = self.robot.get_time()
            self.sense()
            self.robot.sleep(0.05)
            if self.robot.get_time() > 20:
                self.shutdown = True


def main():
    """Main entry point."""
    robot = Robot()
    robot.spin()

def test():
    robot = Robot()
    import leaning_right # or any other data file
    data = leaning_right.get_data()
    robot.robot.load_data_profile(data)
    robot.spin()


if __name__ == "__main__":
    test()