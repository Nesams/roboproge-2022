"""OT05 - Noise."""
import PiBot
import copy
import statistics


class Robot:
    """Robot  class."""

    def __init__(self):
        """Initialize object."""
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.front_middle_laser = None
        self.filter_list = []

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the PiBot reference."""
        self.robot = robot

    def get_front_middle_laser(self) -> float:
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
        """
        Save sensor data into mutators.
        """
        self.front_middle_laser = self.robot.get_front_middle_laser()
        self.filter_list.insert(0, copy.deepcopy(self.front_middle_laser))
        self.filter_list = self.filter_list[:5]

    def plan(self):
        """Devise a plan of action according to the data."""
        pass

    def act(self):
        """Act according to the plan."""
        pass

    def spin(self):
        """The spin loop."""
        while not self.shutdown:
            print(f'Value is {self.get_front_middle_laser()}')
            self.robot.sleep(0.05)
            self.sense()
            self.plan()
            self.act()
            if self.robot.get_time() > 20:
                self.shutdown = True


def main():
    """The main entry point."""
    robot = Robot()
    robot.spin()


def a_test():
    """
    Help.
    """
    robot = Robot()
    import dataset1  # or any other data file
    data = dataset1.get_data()
    robot.robot.load_data_profile(data)
    for i in range(len(data)):
        print(f"laser = {robot.robot.get_front_middle_laser()}")
        robot.robot.sleep(0.05)


if __name__ == "__main__":
    main()
