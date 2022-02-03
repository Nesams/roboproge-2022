"""OT03 - Instantaneous velocity."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class constructor."""
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.left_history = [0]
        self.right_history = [0]
        self.reading_times = [0]



    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set robot reference."""
        self.robot = robot

    def get_left_velocity(self) -> float:
        """
        Return the current left wheel velocity.

        Returns:
          The current wheel translational velocity in meters per second.
        """
        left_change = self.left_history[-1] - self.left_history[-2]
        time_change = self.reading_times[-1] - self.reading_times[-2]
        return (left_change / time_change) * (self.robot.WHEEL_DIAMETER / 180)



    def get_right_velocity(self) -> float:
        """
        Return the current right wheel velocity.

        Returns:
          The current wheel translational velocity in meters per second.
        """
        # Your code here...
        pass

    def sense(self):
        """Read the sensor values from the PiBot API."""
        print(self.robot.get_right_wheel_encoder())
        print(self.robot.get_left_wheel_encoder())
        self.right_history.append(self.robot.get_right_wheel_encoder())
        self.left_history.append(self.robot.get_left_wheel_encoder())
        self.reading_times.append(self.robot.get_time())


    def spin(self):
        """Main loop."""
        while not self.shutdown:
            self.sense()
            timestamp = self.robot.get_time()
            left_velocity = self.get_left_velocity()
            right_velocity = self.get_right_velocity()
            print(f'{timestamp}: {left_velocity} {right_velocity}')
            self.robot.sleep(0.05)
            if self.robot.get_time() > 20:
                self.shutdown = True


def main():
    """Main entry."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()
