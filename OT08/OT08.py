"""OT08 - PID."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class constructor."""
        self.robot = PiBot.PiBot()

        self.prev_right_speed = 0
        self.prev_left_speed = 0

        self.left_wheel_speed = 0
        self.right_wheel_speed = 0

        self.right_wheel_setpoint = 0
        self.left_wheel_setpoint = 0

        self.prev_right_encoder = 0
        self.prev_left_encoder = 0

        self.right_wheel_encoder = 0
        self.left_wheel_encoder = 0

        self.prev_time = 0
        self.time = 0

        self.prev_right_error = 0
        self.prev_left_error = 0

        self.right_error = 0
        self.left_error = 0

        self.right_error_sum = 0
        self.left_error_sum = 0

        self.p = None
        self.i = None
        self.d = None

        self.right_PID = 0
        self.left_PID = 0

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the API reference."""
        self.robot = robot

    def set_pid_parameters(self, p: float, i: float, d: float):
        """
        Set the PID parameters.

        Arguments:
          p -- The proportional component.
          i -- The integral component.
          d -- The derivative component.
        """
        self.p = p
        self.i = i
        self.d = d

    def set_left_wheel_speed(self, speed: float):
        """
        Set the desired setpoint.

        Arguments:
          speed -- The speed setpoint for the controller.
        """
        self.left_wheel_setpoint = speed

    def set_right_wheel_speed(self, speed: float):
        """
        Set the desired setpoint.

        Arguments:
          speed -- The speed setpoint for the controller.
        """
        self.right_wheel_setpoint = speed

    def get_left_wheel_pid_output(self):
        """
        Get the controller output value for the left motor.

        Returns:
          The controller output value.
        """
        return self.right_PID

    def get_right_wheel_pid_output(self):
        """
        Get the controller output value for the right motor.

        Returns:
          The controller output value.
        """
        return self.left_PID

    def get_right_pid(self, time_difference):
        """Calculates the sum of right PID."""
        right_encoder_difference = self.right_wheel_encoder - self.prev_right_encoder
        right_speed = right_encoder_difference / time_difference

        self.prev_right_error = self.right_error

        self.right_error = self.right_wheel_setpoint - right_speed
        p = self.p * self.right_error

        self.right_error_sum = self.right_error
        i = self.i * self.right_error_sum

        right_error_difference = self.right_error - self.prev_right_error
        d = self.d * right_error_difference

        self.right_PID = p + i + d

    def get_left_pid(self, time_difference):
        """Calculates the sum of left PID."""
        left_encoder_difference = self.left_wheel_encoder - self.prev_left_encoder
        left_speed = left_encoder_difference / time_difference

        self.prev_left_error = self.left_error
        self.left_error = self.left_wheel_setpoint - left_speed
        p = self.p * self.left_error

        self.left_error_sum += self.left_error
        i = self.i * self.left_error_sum

        left_error_difference = self.left_error - self.prev_left_error
        d = self.d * left_error_difference

        self.left_PID = p + i + d

    def sense(self):
        """SPA architecture sense block."""
        self.prev_right_speed = self.right_wheel_speed
        self.prev_left_speed = self.left_wheel_speed

        self.right_wheel_encoder = self.robot.get_right_wheel_encoder()
        self.left_wheel_encoder = self.robot.get_left_wheel_encoder()

        self.prev_time = self.time
        self.time = self.robot.get_time()

        time_difference = self.time - self.prev_time
        if time_difference != 0:
            self.get_right_pid(time_difference)
            self.get_left_pid(time_difference)

    def act(self):
        """SPA architecture act block."""
        self.robot.set_right_wheel_speed(self.right_PID)
        self.robot.set_left_wheel_speed(self.left_PID)

    def spin(self):
        """Spin loop."""
        for _ in range(200):
            self.sense()
            self.act()
            self.robot.sleep(0.20)


def main():
    """The main entry point."""
    robot = Robot()
    robot.robot.set_coefficients(1.0, 0.7)
    robot.set_pid_parameters(0.1, 0.04, 0.001)
    robot.set_left_wheel_speed(400)  # degs/s
    robot.set_right_wheel_speed(400)  # degs/s
    robot.spin()


if __name__ == "__main__":
    main()