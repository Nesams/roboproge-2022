"""OT07 - Driving in a Straight Line."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class constructor."""
        self.robot = PiBot.PiBot()
        self.state = "calibrate"
        self.time = 0
        self.right_speed = 8
        self.left_speed = 8

        self.last_right_encoder = 0
        self.last_left_encoder = 0
        self.right_wheel_encoder = 0
        self.left_wheel_encoder = 0
        self.encoder_difference = 0

        self.right_wheel_coefficient = 1.0
        self.left_wheel_coefficient = 1.0

        self.left_wheel_speed = 10
        self.right_wheel_speed = 10

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the robot reference."""
        self.robot = robot

    def set_state(self, state: str):
        """
        Set the current state.

        Arguments:
          state - the state as a string.
        """
        self.state = state

    def get_state(self) -> str:
        """
        Get the state.

        Returns:
          The state as a string.
        """
        return self.state

    def sense(self):
        """The sense method in the SPA architecture."""
        self.right_wheel_encoder = self.robot.get_right_wheel_encoder()
        self.left_wheel_encoder = self.robot.get_left_wheel_encoder()
        self.time = self.robot.get_time()
        if self.state == "calibrate" and self.right_wheel_encoder > self.left_wheel_encoder != 0:
            self.right_wheel_coefficient = 1.0
            self.left_wheel_coefficient = 1.0 + ((self.right_wheel_encoder - self.left_wheel_encoder) // 2) / 10
        elif self.state == "calibrate" and self.left_wheel_encoder > self.right_wheel_encoder != 0:
            self.left_wheel_coefficient = 1.0
            self.right_wheel_coefficient = 1.0 + ((self.left_wheel_encoder - self.right_wheel_encoder) // 2) / 10
        if self.left_wheel_encoder == 0:
            self.left_speed += 1
        if self.right_wheel_encoder == 0:
            self.right_speed += 1

    def plan(self):
        """The plan method in the SPA architecture."""
        if self.time > 3.5 and self.state == "calibrate":
            self.state = "ready"
            self.right_wheel_speed = self.right_speed * self.right_wheel_coefficient
            self.left_wheel_speed = self.left_speed * self.left_wheel_coefficient
        elif self.state == "calibrate":
            self.right_wheel_speed = self.right_speed * self.right_wheel_coefficient
            self.left_wheel_speed = self.left_speed * self.left_wheel_coefficient

    def act(self):
        """The act method in the SPA architecture."""
        if self.state == "drive" or self.state == "calibrate":
            self.robot.set_right_wheel_speed(self.right_wheel_speed)
            self.robot.set_left_wheel_speed(self.left_wheel_speed)


def main():
    """The main entry point."""
    robot = Robot()
    robot.robot.set_coefficients(0.7, 1.0)
    robot.set_state("calibrate")
    for i in range(int(120/0.05)):
        if robot.get_state() == "ready":
            start_left = robot.robot.get_left_wheel_encoder()
            start_right = robot.robot.get_right_wheel_encoder()
            robot.set_state("drive")
        robot.sense()
        robot.plan()
        robot.act()
        robot.robot.sleep(0.05)
    left_delta = robot.robot.get_left_wheel_encoder() - start_left
    right_delta = robot.robot.get_right_wheel_encoder() - start_right
    print(f"left {left_delta} right {right_delta}")
    print(robot.left_wheel_coefficient, robot.right_wheel_coefficient)
    print(robot.right_speed, robot.left_speed)


if __name__ == "__main__":
    main()