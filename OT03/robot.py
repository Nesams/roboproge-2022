"""OT03 - Instantaneous velocity."""
import PiBot


class Robot:
    """The robot class."""

    def __init__(self):
        """Class constructor."""
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
