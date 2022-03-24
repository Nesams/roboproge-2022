"""S1."""
import math
import PiBot


class PIDController:
    """PID-Controller class."""

    def __init__(self, kp: float = 1.0, ki: float = 1.0, kd: float = 1.0, max_integral=1):
        """
        Initialize the PID controller.

        :param kp: Constant multiplier for proportional component.
        :param ki: Constant multiplier for the integral component.
        :param kd: Constant multiplier for the derivative component.
        :param max_integral: Max allowed value for the integral.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.last = 0
        self.integral = 0
        self.max_integral = max_integral

    def get_correction(self, error):
        """
        Calculate the error correction using PID.

        :param error: The difference between desired value and the actual value.
        :return: Force which needs to be applied to get the correct value.
        """
        self.integral += error if error != 0 else -self.integral
        correction = self.kp * error + self.ki * max(min(self.integral * (error != 0), -self.integral), self.integral) \
                     + self.kd * (error - self.last)
        self.last = (self.last + error) / 2
        return correction


class Robot:
    """Robot class."""

    def __init__(self, initial_odometry=[0, 0, 0]):
        """Initialize variables."""
        self.shutdown = False

        self.right_goal_speed = 0
        self.right_wheel_speed = 0
        self.left_goal_speed = 0
        self.left_wheel_speed = 0
        self.robot = PiBot.PiBot()
        self.detected_objects = []

        self.time = 0
        self.last_time = 0
        self.delta_time = 0
        self.right_encoder = 0
        self.left_encoder = 0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.left_delta = 0
        self.right_delta = 0
        self.left_power = 0
        self.right_power = 0

        self.encoder_odometry = initial_odometry.copy()
        self.wheel_radius = self.robot.WHEEL_DIAMETER / 2
        self.angle_goal = 0

        self.camera_resolution = self.robot.CAMERA_RESOLUTION[0]
        self.camera_field_of_view = self.robot.CAMERA_FIELD_OF_VIEW[0]
        self.camera_center = self.camera_resolution / 2

        self.state = "start"
        self.states = {
            "start": self.start,
            "full_scan": self.full_scan,
            "move_to_point": self.move_to_point,
            "drive_forward": self.drive_forward,
            "stop": self.stop
        }
        self.previous_state = None
        self.next_state = None

        self.left_controller = PIDController(0.5, 0.003, 0.002, 5)
        self.right_controller = PIDController(0.5, 0.003, 0.002, 5)

        self.red_coordinates_xy = ()
        self.blue_coordinates_xy = ()

        self.goal_x = 0
        self.goal_y = 0
        self.distance = 0

        self.go_around = False

        self.red_object_angle = None
        self.blue_object_angle = None

        self.red = None
        self.blue = None

        self.red_object = (self.red, self.red_coordinates_xy, self.red_object_angle)
        self.blue_object = (self.blue, self.blue_coordinates_xy, self.blue_object_angle)

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the API reference."""
        self.robot = robot

    def start(self):
        """Grabbers down."""
        self.robot.set_grabber_height(0)
        self.next_state = "full_scan"

    def full_scan(self):
        self.cameradetection()
        if self.previous_state == "full_scan":
            if self.encoder_odometry[2] > math.radians(358):
                self.next_state = "move_to_point"
            else:
                self.drive(2, 1)
                self.next_state = "full_scan"

    def move_to_point(self):
        if self.previous_state == "full_scan":
            if 0 < (self.blue_object_angle - self.red_object_angle) <= 180:
                self.go_around = False
                self.angle_goal = (self.blue_object_angle - self.red_object_angle) / 2
                self.goal_x = (self.red_coordinates_xy[0] + self.blue_coordinates_xy[0]) / 2
                self.goal_y = (self.red_coordinates_xy[1] + self.blue_coordinates_xy[1]) / 2
            elif (self.blue_object_angle - self.red_object_angle) <= -180:
                angle = (self.blue_object_angle - self.red_object_angle) / 2
                if angle < 180:
                    self.angle_goal = angle + 180
                else:
                    self.angle_goal = angle - 180
                self.go_around = False
                self.goal_x = (self.red_coordinates_xy[0] + self.blue_coordinates_xy[0]) / 2
                self.goal_y = (self.red_coordinates_xy[1] + self.blue_coordinates_xy[1]) / 2
            else:
                self.angle_goal = self.red_object_angle + 15
                if self.angle_goal >= 360:
                    self.angle_goal -= 360
                self.go_around = True
                distance = 2 * math.sqrt(((self.red_coordinates_xy[0] - self.encoder_odometry[0]) ** 2) + ((self.red_coordinates_xy[1] - self.encoder_odometry[1]) ** 2))
                self.goal_x = self.encoder_odometry[0] + (distance * math.cos(self.angle_goal))
                self.goal_y = self.encoder_odometry[1] + (distance * math.sin(self.angle_goal))
            self.next_state = "move_to_point"
        else:
            if self.angle_goal - math.radians(5) <= self.encoder_odometry[2] - math.radians(180) <= self.angle_goal + math.radians(5):
                self.next_state = "drive_forward"
            else:
                self.drive(2, 1)
                self.next_state = "move_to_point"

    def drive_forward(self):
        if self.goal_x - 0.05 < self.encoder_odometry[0] < self.goal_x + 0.05 and self.goal_y - 0.05 < self.encoder_odometry[1] < self.goal_y + 0.05:
            if self.go_around:
                self.next_state = "full_scan"
            else:
                self.next_state = "stop"
        else:
            self.drive(2, 0)
            self.next_state = "drive_forward"

    def stop(self):
        """Default end state."""
        if self.previous_state != "stop":
            self.drive(0)
            self.next_state = "stop"
        else:
            quit()

    def drive(self, speed: int, turning_rate: float = 0):
        """
        Drive the robot at given speed and rotation.

        :param speed: Speed of the faster motor.
        :param turning_rate: The rate that the robot will turn.
        Range of -1 to 1
        0.5 Will turn around a single wheel.
        :return:
        """
        self.left_goal_speed = min(1 - 2 * turning_rate, 1) * speed
        print(self.left_goal_speed)
        print(self.left_wheel_speed)
        self.right_goal_speed = min(1 + 2 * turning_rate, 1) * speed
        print(self.right_goal_speed)
        print(self.right_wheel_speed)

    def calculate_motor_power(self):
        """
        Calculate the power for both motor to achieve the desired speed.
        :return: None
        """
        left_error = self.left_wheel_speed - self.left_goal_speed
        right_error = self.right_wheel_speed - self.right_goal_speed
        self.left_power -= round(self.left_controller.get_correction(left_error))
        self.right_power -= round(self.right_controller.get_correction(right_error))
        if self.left_goal_speed == 0 and self.right_goal_speed == 0:
            self.left_power = 0
            self.right_power = 0

    def cameradetection(self):
        """Calculating the closest object angle."""
        if len(self.detected_objects) == 0:
            pass
        for object in self.detected_objects:
            print(object)
            if object[0] == 'red sphere':
                self.red_coordinates_xy = object[1]
                red_coordinates_x = self.red_coordinates_xy[0]
                red_x_difference = self.camera_center - red_coordinates_x
                red_object_angle = (red_x_difference / self.camera_resolution) * self.camera_field_of_view
                red_object_angle = (red_object_angle * math.pi) / 180
                self.red_object_angle = red_object_angle + self.encoder_odometry[2]
                print(self.red_object_angle)
            if object[0] == 'blue sphere':
                self.blue_coordinates_xy = object[1]
                blue_coordinates_x = self.blue_coordinates_xy[0]
                blue_x_difference = self.camera_center - blue_coordinates_x
                blue_object_angle = (blue_x_difference / self.camera_resolution) * self.camera_field_of_view
                blue_object_angle = (blue_object_angle * math.pi) / 180
                self.blue_object_angle = blue_object_angle + self.encoder_odometry[2]
                print(self.blue_object_angle)

    def sense(self):
        """SPA architecture sense block."""
        # Read the current time
        self.last_time = self.time
        self.time = self.robot.get_time()
        self.delta_time = self.time - self.last_time

        # Read wheel encoder values
        self.last_left_encoder = self.left_encoder
        self.left_encoder = self.robot.get_left_wheel_encoder()
        self.last_right_encoder = self.right_encoder
        self.right_encoder = self.robot.get_right_wheel_encoder()
        self.left_delta = self.left_encoder - self.last_left_encoder
        self.right_delta = self.right_encoder - self.last_right_encoder

        self.detected_objects = self.robot.get_camera_objects()

        # Odometry
        if self.delta_time > 0:
            self.left_wheel_speed = math.radians(self.left_delta) / self.delta_time
            self.right_wheel_speed = math.radians(self.right_delta) / self.delta_time
            self.encoder_odometry[2] = (self.wheel_radius / self.robot.AXIS_LENGTH) * (
                        math.radians(self.right_encoder) - math.radians(self.left_encoder))
            self.encoder_odometry[0] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.cos(
                self.encoder_odometry[2]) * self.delta_time
            self.encoder_odometry[1] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.sin(
                self.encoder_odometry[2]) * self.delta_time

    def plan(self):
        print("State: ", self.state)
        self.states[self.state]()
        self.previous_state = self.state
        self.state = self.next_state

    def act(self):
        self.calculate_motor_power()
        self.robot.set_left_wheel_speed(self.left_power)
        self.robot.set_right_wheel_speed(self.right_power)

    def spin(self):
        """The spin loop."""
        while not self.shutdown:
            self.sense()
            self.plan()
            self.act()
            self.robot.sleep(0.05)
            if self.robot.get_time() > 600:
                self.shutdown = True

def main():
    """Main  entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()
