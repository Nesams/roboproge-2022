import PiBot


class Robot:
    def __init__(self):
        """
        Initialize the robot.
        """
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.front_middle_laser = 0
        self.state = "unknown"

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """
        Set the reference to PiBot object.

        Returns:
          None
        """
        self.robot = robot

    def get_state(self) -> str:
        """
        Return the current state.

        Returns:
          The current state as a string.
        """
        return self.state

    def sense(self):
        """
        Read values from sensors via PiBot  API into class variables (self).

        Returns:
          None
        """
        self.front_middle_laser = self.robot.get_front_middle_laser()

    def plan(self):
        """
        Perform the planning steps as required by the problem statement.

        Returns:
          None
        """
        if 0 < self.front_middle_laser <= 0.5:
            self.state = 'close'
        elif 1.5 >= self.front_middle_laser > 0.5:
            self.state = 'ok'
        elif 2.0 > self.front_middle_laser > 1.5:
            self.state = 'far'
        elif self.front_middle_laser >= 2.0:
            self.state = 'very far'
        else:
            self.state = 'unknown'

    def spin(self):
        """
        The main loop of the robot.
        """
        while not self.shutdown:
            print(f'The time is {self.robot.get_time()}!')
            self.robot.sleep(0.05)
            if self.robot.get_time() > 20:
                self.sense()
                self.plan()
                self.get_state()
                self.shutdown = True

    # Add more code...


def main():
    """
    Create a Robot object and spin it.
    """
    robot = Robot()
    robot.spin()

def test():
    """
    Use this while developing your solution
    (change main() to test() below in the "if name main")!
    You need to get the PiBot.py and forward_reverse.py files
    from the test data repository:
    https://gitlab.cs.ttu.ee/iti0201-2021/ex-data

    NB! There is a similar test/development helper for
        all the following OT exercises.
        Always check the "test data" repository.
    """
    robot = Robot()
    import forward_reverse # or any other data file
    data = forward_reverse.get_data()
    robot.robot.load_data_profile(data)
    for i in range(len(data)):
        robot.sense()
        robot.plan()
        print(f"middle_laser = {robot.robot.get_front_middle_laser()}")
        robot.robot.sleep(0.05)


if __name__ == "__main__":
        test()
        #main()