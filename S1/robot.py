"""S1."""
import math
import PiBot


class PIDController:
    """PID-Controller class."""

    def __init__(self, kp: float = 0.4, ki: float = 0.04, kd: float = 0.001, max_integral=1):
        """
        Initialize the PID controller.

        :param kp: Constant multiplier for proportional component.
        :param ki: Constant multiplier for the integral component.
        :param kd: Constant multiplier for the derivative component.
        :param max_integral: Max allowed value for the integral.
        """
        self.error_sum = 0
        self.previous_error = 0
        self.desired_speed = 0
        self.pid = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last = 0
        self.integral = 0
        self.max_integral = max_integral
        self.last_pid = 0
        self.pid_output = 0
        self.last_encoder = 0
        self.wheel_distance = 0

    def set_desired_pid_speed(self, speed: float):
        self.error_sum = 0
        self.desired_speed = speed

    def reset(self):
        self.pid = 0
        self.error_sum = 0
        self.previous_error = 0
        self.desired_speed = 0
        self.pid_output = 0
        self.last_encoder = 0
        self.wheel_distance = 0

    def increment(self, previous_pid, new_pid):
        if abs(abs(previous_pid) - abs(new_pid)) > 1:
            if previous_pid - new_pid < 0:
                return 1
            else:
                return -1
        else:
            return new_pid - previous_pid

    def update_pid(self, encoder, delta_time):
        self.last_pid = self.pid
        self.wheel_distance = (encoder - self.last_encoder) / delta_time
        self.last_encoder = encoder
        if abs(self.wheel_distance) > 0:
            low = self.last_pid
        else:
            low = 0

        if delta_time != 0:
            error = self.desired_speed - self.wheel_distance
            self.error_sum += error * delta_time
            error_difference = (error - self.previous_error) / delta_time
            self.previous_error = error
            self.pid = self.kp * error + self.ki * self.error_sum + self.kd * error_difference
            self.pid_output += self.increment(self.last_pid, self.pid)
            if self.desired_speed > 0:
                self.pid_output = max(self.pid_output, low)
            else:
                self.pid_output = min(self.pid_output, -low)

    def get_pid_output(self):
        return self.pid_output


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

        self.encoder_odometry = initial_odometry.copy()
        self.wheel_radius = self.robot.WHEEL_DIAMETER / 2
        self.angle_goal = 0

        self.camera_resolution = self.robot.CAMERA_RESOLUTION[0]
        self.camera_y_resolution = self.robot.CAMERA_RESOLUTION[1]
        self.camera_field_of_view = self.robot.CAMERA_FIELD_OF_VIEW[0]
        self.camera_center = self.camera_resolution / 2

        self.state = "start"

        self.left_controller = PIDController(0.0001, 0.05, 0.0001, 5)
        self.right_controller = PIDController(0.0001, 0.05, 0.0001, 5)

        self.red_coordinates_xy = ()
        self.blue_coordinates_xy = ()
        self.blue_distance = None
        self.red_distance = None
        self.red_x = None
        self.red_y = None
        self.blue_x = None
        self.blue_y = None

        self.goal_x = 0
        self.goal_y = 0
        self.distance = 0
        self.goal_distance = None
        self.start_x = None
        self.start_y = None
        self.rotation = 0.0
        self.rotation_raw = 0.0
        self.rotation_base = 0.0

        self.go_around = False
        self.state_switch = True
        self.red_object_angle = None
        self.blue_object_angle = None


        self.red = None
        self.blue = None

        self.red_object = (self.red, self.red_coordinates_xy, self.red_object_angle)
        self.blue_object = (self.blue, self.blue_coordinates_xy, self.blue_object_angle)

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the API reference."""
        self.robot = robot

    def normalize_angle(self, angle):
        """Normalizing angle. Range(0...360)."""
        while angle < 0:
            angle += 360
        while angle > 360:
            angle -= 360
        return angle

    def start(self):
        """Grabbers down."""
        self.robot.set_grabber_height(0)
        self.state = "full_scan"

    def full_scan(self):
        self.cameradetection()
        print("state switch boolean", self.state_switch)
        if self.state_switch is False:
            if self.red_object_angle is not None and self.blue_object_angle is not None and self.encoder_odometry[2] >= 355:
                if 175 < self.red_object_angle - self.blue_object_angle < 185 or 175 < self.blue_object_angle - self.red_object_angle < 185:
                    self.state = "stop"
                    self.right_controller.set_desired_pid_speed(0)
                    self.left_controller.set_desired_pid_speed(0)
                else:
                    self.state = "move_to_point"
                    self.state_switch = True
                    self.right_controller.set_desired_pid_speed(0)
                    self.left_controller.set_desired_pid_speed(0)
        else:
            self.blue_object_angle = None
            self.red_object_angle = None
            self.state_switch = False
            self.left_controller.set_desired_pid_speed(-100)
            self.right_controller.set_desired_pid_speed(100)

    def calculate_object_center(self):
        if 0 <= (self.blue_object_angle - self.red_object_angle) <= 180:
            # if 0 degrees is not between the two spheres and blue is on right side
            # print("both spheres are on one side of 0 and correct orientation")
            self.go_around = False
            self.angle_goal = self.normalize_angle((self.red_object_angle + self.blue_object_angle) / 2)
            self.goal_x = (self.red_x + self.blue_x) / 2
            self.goal_y = (self.red_y + self.blue_y) / 2
            self.goal_distance = math.sqrt(
                ((self.goal_x - self.encoder_odometry[0]) ** 2) + ((self.goal_y - self.encoder_odometry[1]) ** 2))
        elif (self.blue_object_angle - self.red_object_angle) <= -180:
            # print("both spheres are on different sides of 0 and correct orientation")
            # if 0 degrees is between the two spheres and blue is on the right side
            self.angle_goal = self.normalize_angle((self.blue_object_angle + self.red_object_angle) / 2 + 180)
            self.go_around = False
            self.goal_x = (self.red_x + self.blue_x) / 2
            self.goal_y = (self.red_y + self.blue_y) / 2
            self.goal_distance = math.sqrt(
                ((self.goal_x - self.encoder_odometry[0]) ** 2) + ((self.goal_y - self.encoder_odometry[1]) ** 2))
        else:
            # print("wrong orientation")
            self.angle_goal = self.normalize_angle(self.red_object_angle + 15)
            self.go_around = True
            self.goal_distance = 2 * math.sqrt(
                ((self.red_x - self.encoder_odometry[0]) ** 2) + ((self.red_y - self.encoder_odometry[1]) ** 2))

    def move_to_point(self):
        if self.state_switch is True:
            self.state_switch = False
            self.left_controller.set_desired_pid_speed(-100)
            self.right_controller.set_desired_pid_speed(100)
            self.calculate_object_center()
        else:
            print("Get rotation:  ", self.encoder_odometry[2])
            print("angle goal for turning", self.angle_goal)
            if self.angle_goal - 10 <= self.encoder_odometry[2] <= self.angle_goal + 2:
                self.state = "drive_forward"
                self.state_switch = True
                self.right_controller.set_desired_pid_speed(0)
                self.left_controller.set_desired_pid_speed(0)
                self.left_controller.reset()
                self.right_controller.reset()

    def drive_forward(self):
        print("Angles:  ", self.angle_goal, self.encoder_odometry[2])
        if self.state_switch is True:
            self.state_switch = False
            self.start_x = self.encoder_odometry[0]
            self.start_y = self.encoder_odometry[1]
            self.left_controller.set_desired_pid_speed(100)
            self.right_controller.set_desired_pid_speed(100)
        print("goal distance: ", self.goal_distance)
        distance_traveled = math.sqrt((self.encoder_odometry[0] - self.start_x) ** 2 + (self.encoder_odometry[1] - self.start_y) ** 2)
        print("Distance traveled: ", distance_traveled)
        print("encoder odom x", self.encoder_odometry[0])
        print("encoder odom y", self.encoder_odometry[1])
        if distance_traveled >= self.goal_distance:
            self.state = "full_scan"
            self.state_switch = True

    def get_rotation(self):
        """Getter method."""
        return abs(self.rotation)

    def stop(self):
        """Default end state."""
        self.left_controller.set_desired_pid_speed(0)
        self.right_controller.set_desired_pid_speed(0)

    def cameradetection(self):
        """Calculating the closest object angle."""
        if len(self.detected_objects) == 0:
            pass
        for object in self.detected_objects:
            #print(object)
            #print("Camera resolution: ", self.robot.CAMERA_RESOLUTION[1])
            if object[0] == 'red sphere' and not self.red_object_angle:
                self.red_coordinates_xy = object[1]
                red_coordinates_x = self.red_coordinates_xy[0]
                if self.camera_center - 50 < red_coordinates_x < self.camera_center + 50:
                    print("got red!")
                    self.red_distance = self.robot.get_front_middle_laser()
                    print(self.red_distance)
                    red_x_difference = self.camera_center - red_coordinates_x
                    red_object_angle = (red_x_difference / self.camera_resolution) * self.camera_field_of_view
                    self.red_object_angle = self.normalize_angle(red_object_angle + self.encoder_odometry[2])
                    self.red_x = self.red_distance * math.cos(math.radians(self.red_object_angle))
                    self.red_y = self.red_distance * math.sin(math.radians(self.red_object_angle))
                # self.red_distance = 16 / object[2]
                #print("Red object angle: ", self.red_object_angle_deg)
                #print("robot angle: ", self.get_rotation())
            if object[0] == 'blue sphere' and not self.blue_object_angle:
                self.blue_coordinates_xy = object[1]
                blue_coordinates_x = self.blue_coordinates_xy[0]
                if self.camera_center - 50 < blue_coordinates_x < self.camera_center + 50:
                    print("got blue!")
                    self.blue_distance = self.robot.get_front_middle_laser()
                    print(self.blue_distance)
                    blue_x_difference = self.camera_center - blue_coordinates_x
                    blue_object_angle = (blue_x_difference / self.camera_resolution) * self.camera_field_of_view
                    self.blue_object_angle = self.normalize_angle(blue_object_angle + self.encoder_odometry[2])
                    self.blue_x = self.blue_distance * math.cos(math.radians(self.blue_object_angle))
                    self.blue_y = self.blue_distance * math.sin(math.radians(self.blue_object_angle))
                # self.blue_distance = 16 / object[2]
                #print("Blue object angle: ", self.blue_object_angle_deg)
                #print("robot angle: ", self.get_rotation())

    def reset_rotation(self):
        """Reset rotation."""
        self.rotation_base = self.rotation_raw

    def sense(self):
        """SPA architecture sense block."""
        # Read the current time
        self.last_time = self.time
        self.time = self.robot.get_time()
        self.delta_time = self.time - self.last_time

        self.last_left_encoder = self.left_encoder
        self.last_right_encoder = self.right_encoder
        # Read wheel encoder values
        self.left_encoder = self.robot.get_left_wheel_encoder()
        self.right_encoder = self.robot.get_right_wheel_encoder()

        # Rotation
        self.rotation_raw = self.robot.get_rotation()
        self.rotation = self.rotation_raw - self.rotation_base
        if self.get_rotation() > 360:
            self.reset_rotation()
        # Camera
        if self.state == "full_scan":
            self.detected_objects = self.robot.get_camera_objects()

        # Odometry
        if self.delta_time > 0:
            self.left_wheel_speed = math.radians(self.left_encoder - self.last_left_encoder) / self.delta_time
            self.right_wheel_speed = math.radians(self.right_encoder - self.last_right_encoder) / self.delta_time
            print("Left wheel power: ", self.left_controller.get_pid_output())
            print("Right wheel power: ", self.right_controller.get_pid_output())
            self.encoder_odometry[2] = self.robot.get_rotation()
            self.encoder_odometry[0] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.cos(
                math.radians(self.encoder_odometry[2])) * self.delta_time
            self.encoder_odometry[1] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.sin(
                math.radians(self.encoder_odometry[2])) * self.delta_time
            self.encoder_odometry[2] = self.normalize_angle(self.encoder_odometry[2])

        self.left_controller.update_pid(self.left_encoder, self.delta_time)
        self.right_controller.update_pid(self.right_encoder, self.delta_time)

    def plan(self):
        if self.state == "start":
            self.start()
        elif self.state == "full_scan":
            self.full_scan()
        elif self.state == "move_to_point":
            self.move_to_point()
        elif self.state == "drive_forward":
            self.drive_forward()
        elif self.state == "stop":
            self.stop()
        print("State: ", self.state)

    def act(self):
        self.robot.set_left_wheel_speed(self.left_controller.get_pid_output())
        self.robot.set_right_wheel_speed(self.right_controller.get_pid_output())

    def spin(self):
        """The spin loop."""
        while not self.shutdown:
            self.sense()
            self.plan()
            self.act()
            self.robot.sleep(0.05)


def main():
    """Main  entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()
