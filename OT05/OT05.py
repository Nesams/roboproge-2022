"""OT05 - Noise."""
import statistics

import PiBot

class Robot:
    """Robot class."""

    def __init__(self):
        """Initialize object."""
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.line_directions = []
        self.front_middle_laser = None
        self.filter_list = []

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the PiBot reference."""
        self.robot = robot

    def get_front_middle_laser(self):
        """
        Return the filtered value.

        Returns:
          None if filter is empty, filtered value otherwise.
        """
        if not self.filter_list:
            return None
        else:
            return statistics.median(self.filter_list)

    def sense(self):
        self.front_middle_laser = self.get_front_middle_laser()
        self.filter_list.insert(0, copy.deepcopy(self.front_middle_laser))
        self.filter_list = self.filter_list[:5]

    def spin(self):
        """The spin loop."""
        while not self.shutdown:
            print(f'Value is {self.get_front_middle_laser()}')
            self.robot.sleep(0.05)
            if self.robot.get_time() > 20:
                self.shutdown = True


def main():
    """The main entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()
