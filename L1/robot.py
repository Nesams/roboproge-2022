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
        self.SLEEP_TIME = 0.025
        # PID
        self.time = 0
        self.previous_time = 0
        self.dt = 0
        self.right_encoder = 0
        self.left_encoder = 0
        self.previous_left_encoder = 0
        self.previous_right_encoder = 0

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set robot reference."""
        self.robot = robot

    def sense(self):
        """Sense method as per SPA architecture."""
        self.line_directions = [self.robot.get_leftmost_line_sensor(), self.robot.get_second_line_sensor_from_left(),
                                self.robot.get_third_line_sensor_from_left(), self.robot.get_third_line_sensor_from_right(),
                                self.robot.get_second_line_sensor_from_right(), self.robot.get_rightmost_line_sensor()]

        self.previous_right_encoder = self.right_encoder
        self.previous_left_encoder = self.left_encoder
        self.left_encoder = self.robot.get_left_wheel_encoder()
        self.right_encoder = self.robot.get_right_wheel_encoder()

        self.previous_time = self.time
        self.time = self.robot.get_time()
        self.dt = self.time - self.previous_time

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
        elif directions == -2:
            self.motors = [-self.DRIVE_SPEED, -self.DRIVE_SPEED]
        self.compensate_speed()

    def compensate_speed(self):
        """Adjust speeds for realism."""
        left_speed = self.motors[0]
        right_speed = self.motors[1]
        real_speed_left = self.get_left_wheel_speed()
        real_speed_right = self.get_right_wheel_speed()

        print("before adj left", left_speed)
        print("before adj right", right_speed)
        print("real left:", real_speed_left)
        print("real right:", real_speed_right)
        # if real speed is zero it is not adjusted rn
        if abs(real_speed_left) != abs(real_speed_right):
            # Forward
            if real_speed_left > 0 and real_speed_right > 0:
                if real_speed_left < real_speed_right:
                    difference = real_speed_right - real_speed_left
                    left_speed += round(difference / (real_speed_right / self.DRIVE_SPEED))
                else:
                    difference = real_speed_left - real_speed_right
                    right_speed += round(difference / (real_speed_left / self.DRIVE_SPEED))

            # Turn right
            elif real_speed_left > 0 and real_speed_right < 0:
                if real_speed_left > abs(real_speed_right):
                    difference = real_speed_left - abs(real_speed_right)
                    right_speed -= round(difference / (real_speed_left / self.DRIVE_SPEED))
                else:
                    difference = abs(real_speed_right - real_speed_left)
                    left_speed += round(difference / (abs(real_speed_right) / self.DRIVE_SPEED))

            # Turn left
            elif real_speed_left < 0 and real_speed_right > 0:
                if abs(real_speed_left) < real_speed_right:
                    difference = real_speed_right - abs(real_speed_left)
                    left_speed -= round(difference / (real_speed_right / self.DRIVE_SPEED))
                else:
                    difference = abs(real_speed_left) - real_speed_right
                    right_speed += round(difference / (abs(real_speed_left) / self.DRIVE_SPEED))

            # Backwards
            elif real_speed_left < 0 and real_speed_right < 0:
                if real_speed_left < real_speed_right:
                    difference = abs(real_speed_left - real_speed_right)
                    right_speed -= round(difference / (abs(real_speed_left) / self.DRIVE_SPEED))
                else:
                    difference = abs(real_speed_right - real_speed_left)
                    left_speed -= round(difference / (abs(real_speed_right) / self.DRIVE_SPEED))

            self.motors[0] = left_speed
            self.motors[1] = right_speed

        print("new left:", left_speed)
        print("new right:", right_speed)

    def get_left_wheel_speed(self) -> float:
        """Calculate the left wheel speed."""
        if self.dt != 0:
            return (self.left_encoder - self.previous_left_encoder) / self.dt
        else:
            return 0

    def get_right_wheel_speed(self) -> float:
        """Calculate the right wheel speed."""
        if self.dt != 0:
            return (self.right_encoder - self.previous_right_encoder) / self.dt
        else:
            return 0

    def act(self):
        """Act method."""
        # TODO limit max speed if necessary
        limit = 99
        if self.motors[0] > limit:
            self.motors[0] = limit
        elif self.motors[0] < -limit:
            self.motors[0] = -limit
        if self.motors[1] > limit:
            self.motors[1] = limit
        elif self.motors[1] < -limit:
            self.motors[1] = -limit
        self.robot.set_left_wheel_speed(self.motors[0])
        self.robot.set_right_wheel_speed(self.motors[1])

    def get_line_direction(self):
        """
        Return the direction of the line based on sensor readings.

        Returns:
          -1: Line is on the right (i.e., the robot should turn right to reach the line again)
           0: Robot is on the line (i.e., the robot should not turn to stay on the line) or no sensor info
           1: Line is on the left (i.e., the robot should turn left to reach the line again)
           -2: backwards
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
            if self.last_positions[-1] == 0:
                return -2
            return self.last_positions[-1]
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
        else:
            return self.last_positions[-1]

    def spin(self):
        """The main spin loop."""
        while not self.shutdown:
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