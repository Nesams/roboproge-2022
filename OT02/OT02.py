import PiBot


class Robot:
    def __init__(self):
        """
        Initialize the robot.
        """
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.front_right_laser = 0
        self.front_left_laser = 0
        self.front_middle_laser = 0
        self.distance_sensor = 0
        self.first_line_sensor_from_right = 0
        self.second_line_sensor_from_right = 0
        self.third_line_sensor_from_right = 0
        self.first_line_sensor_from_left = 0
        self.second_line_sensor_from_left = 0
        self.third_line_sensor_from_left = 0
        self.left_straight_ir = 0
        self.left_diagonal_ir = 0
        self.left_side_ir = 0
        self.right_straight_ir = 0
        self.right_diagonal_ir = 0
        self.right_side_ir = 0

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
        # Your code here...
        pass

    def sense(self):
        """
        Read values from sensors via PiBot  API into class variables (self).

        Returns:
          None
        """
        # Write some code here...
        self.front_right_laser = self.robot.get_front_right_laser()
        self.front_left_laser = self.robot.get_front_left_laser()
        self.front_middle_laser = self.robot.get_front_middle_laser()
        self.distance_sensor = self.robot.get_distance_sensor()
        self.first_line_sensor_from_right = self.robot.get_rightmost_line_sensor()
        self.second_line_sensor_from_right = self.robot.get_second_line_sensor_from_right()
        self.third_line_sensor_from_right = self.robot.get_third_line_sensor_from_right()
        self.first_line_sensor_from_left = self.robot.get_leftmost_line_sensor()
        self.second_line_sensor_from_left = self.robot.get_second_line_sensor_from_left()
        self.third_line_sensor_from_left = self.robot.get_third_line_sensor_from_left()
        self.left_straight_ir = self.robot.get_rear_left_straight_ir()
        self.left_diagonal_ir = self.robot.get_rear_left_diagonal()
        self.left_side_ir = self.robot.get_rear_left_side_ir()
        self.right_straight_ir = self.robot.get_rear_right_straight_ir()
        self.right_diagonal_ir = self.robot.get_rear_right_diagonal_ir()
        self.right_side_ir = self.robot.get_rear_right_side_ir()

    def plan(self):
        """
        Perform the planning steps as required by the problem statement.

        Returns:
          None
        """
        # Write some code here...
        pass

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
    main()