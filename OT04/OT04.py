"""OT04 - Line tracking."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class initialization."""
        self.robot = PiBot.PiBot()
        self.shutdown = False

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set robot reference."""
        self.robot = robot

    def sense(self):
        """Sense method as per SPA architecture."""
        # Your code here...
        pass

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