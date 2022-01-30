import PiBot


class Robot:
    def __init__(self):
        """
        Initialize class.
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
        Set the reference to the robot instance.

        NB! This is required for automatic testing.
        You are not expected to call this method in your code.

        Arguments:
          robot -- the reference to the robot instance.
        """
        self.robot = robot

    def spin(self):
        """
        The main loop of the robot.
        This loop is expected to call sense, plan, act methods cyclically.
        """
        while not self.shutdown:
            print(f'The time is {self.robot.get_time()}!')
            self.robot.sleep(0.05)
            if self.robot.get_time() > 20:
                self.shutdown = True

    def sense(self):
        """
        Function to get sensors data.
        :return:
        """
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
        Function that analyzes data.
        :return:
        """
        return None

    def act(self):
        """
        Function that moves the Robot.
        :return:
        """


def main():
    """
    The main function.

    Create a Robot class object and run it.
    """
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()