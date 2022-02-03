"""OT03 - Instantaneous velocity."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class constructor."""
        self.robot = PiBot.PiBot()
        self.shutdown = False

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set robot reference."""
        self.robot = robot

    def get_left_velocity(self) -> float:
        """
        Return the current left wheel velocity.

        Returns:
          The current wheel translational velocity in meters per second.
        """
        # Your code here...
        pass

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
        # Your code here...
        pass

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
