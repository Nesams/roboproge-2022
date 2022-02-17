"""OT04 - Line tracking."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class initialization."""
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.line_directions = []
        self.last_positions = [0]
        self.motors = []
        self.TURN_SPEED = 8
        self.DRIVE_SPEED = 8
        self.SLEEP_TIME = 0.05

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set robot reference."""
        self.robot = robot

    def sense(self):
        """Sense method as per SPA architecture."""
        self.line_directions = [self.robot.get_leftmost_line_sensor(), self.robot.get_second_line_sensor_from_left(),
                                self.robot.get_third_line_sensor_from_left(), self.robot.get_third_line_sensor_from_right(),
                                self.robot.get_second_line_sensor_from_right(), self.robot.get_rightmost_line_sensor()]

    def plan(self):
        """Plan method."""
        directions = self.get_line_direction()
        self.motors.clear()
        if directions == -1:  # Right
            self.motors = [self.TURN_SPEED, -self.TURN_SPEED]
        elif directions == 0:  # On line
            self.motors = [self.DRIVE_SPEED, self.DRIVE_SPEED]
        elif directions == 1:  # Left
            self.motors = [-self.TURN_SPEED, self.TURN_SPEED]

    def act(self):
        """Act method."""
        self.robot.set_left_wheel_speed(self.motors[0])
        self.robot.set_right_wheel_speed(self.motors[1])

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
        print(self.line_directions)
        print(line_exists)
        print("")
        if line_exists.count(1) == 0:
            # if len(self.last_positions) >= 2:
            #     return self.last_positions[-1] if self.last_positions[-1] != 0 else self.last_positions[-2]
            # else:
            return -1
        if line_exists[1:5].count(1) == 3:
            return 0
        if line_exists[:3].count(1) > line_exists[3:].count(1):
            self.last_positions.append(1)
            return 1
        if line_exists[:3].count(1) < line_exists[3:].count(1):
            self.last_positions.append(-1)
            return -1
        if line_exists[:3].count(1) == line_exists[3:].count(1):
            self.last_positions.append(0)
            return 0
        # else:
        #     return self.last_positions[-1]

    def spin(self):
        """The main spin loop."""
        while not self.shutdown:
            timestamp = self.robot.get_time()
            self.sense()
            self.plan()
            self.act()
            self.robot.sleep(self.SLEEP_TIME)


def main():
    """Main entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()