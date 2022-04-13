"""S1."""
import math
import PiBot


class PIDController:
    """PID-Controller class."""

    def __init__(self, kp: float = 0.001, ki: float = 0.04, kd: float = 0.001, max_integral=1):
        """
        Initialize the PID controller.

        :param kp: Constant multiplier for proportional component.
        :param ki: Constant multiplier for the integral component.
        :param kd: Constant multiplier for the derivative component.
        :param max_integral: Max allowed value for the integral.
        """
        self.wheel_error_sum = 0
        self.wheel_previous_error = 0
        self.wheel_distance_ref = 0
        self.pid = 0
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.last = 0
        self.integral = 0
        self.max_integral = max_integral
        self.last_pid = 0
        self.pid_output = 0

    def set_desired_pid_speed(self, speed: float):
        self.wheel_distance_ref = speed

    def reset(self):
        self.pid = 0
        self.wheel_error_sum = 0
        self.wheel_previous_error = 0
        self.wheel_distance_ref = 0

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

    def increment(self, previous_pid, new_pid):
        increase = 1
        if abs(abs(previous_pid) - abs(new_pid)) > increase:
            if previous_pid - new_pid < 0:
                return increase
            else:
                return -increase
        else:
            return new_pid - previous_pid

    def update_pid(self):
        self.last_pid = self.pid
        self.wheel_error_sum += self.wheel_previous_error
        self.wheel_previous_error = self.wheel_distance_ref
        self.pid = self.kp * self.wheel_distance_ref + self.ki * self.wheel_error_sum
        self.pid_output += self.increment(self.last_pid, self.pid)


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
        self.left_power = 0
        self.right_power = 0
        self.forward_start_time = None

        self.encoder_odometry = initial_odometry.copy()
        self.wheel_radius = self.robot.WHEEL_DIAMETER / 2
        self.angle_goal = 0

        self.camera_resolution = self.robot.CAMERA_RESOLUTION[0]
        self.camera_y_resolution = self.robot.CAMERA_RESOLUTION[1]
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

        self.left_controller = PIDController(0.001, 0.04, 0.001, 5)
        self.right_controller = PIDController(0.001, 0.04, 0.001, 5)


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

        self.red_object_angle = None
        self.blue_object_angle = None

        self.red_object_angle_deg = None
        self.blue_object_angle_deg = None

        self.angle_goal_deg = None

        self.red = None
        self.blue = None

        self.red_object = (self.red, self.red_coordinates_xy, self.red_object_angle)
        self.blue_object = (self.blue, self.blue_coordinates_xy, self.blue_object_angle)

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the API reference."""
        self.robot = robot

    def normalize_angle(self, angle):
        """Normalizing angle. Range(π...2π)."""
        while angle < 0:
            angle += 2 * math.pi
        while angle > 2 * math.pi:
            angle -= 2 * math.pi
        return angle

    def start(self):
        """Grabbers down."""
        self.robot.set_grabber_height(0)
        self.next_state = "full_scan"

    def full_scan(self):
        self.cameradetection()
        if self.previous_state == "full_scan":
            if self.get_rotation() > 350 and self.red_object_angle is not None and self.blue_object_angle is not None:
                self.next_state = "move_to_point"
                self.right_controller.set_desired_pid_speed(0)
                self.left_controller.set_desired_pid_speed(0)
                #self.drive(0, 0)
            else:
                self.left_controller.set_desired_pid_speed(-15)
                self.right_controller.set_desired_pid_speed(15)
                #self.drive(1, 1)
                self.next_state = "full_scan"
        else:
            self.left_controller.reset()
            self.right_controller.reset()

    def move_to_point(self):
        if self.previous_state == "full_scan":
            self.left_controller.reset()
            self.right_controller.reset()
            if math.radians(0) <= (self.blue_object_angle - self.red_object_angle) <= math.radians(180):
                # if 0 degrees is not between the two spheres and blue is on right side
                #print("both spheres are on one side of 0 and correct orientation")
                self.go_around = False
                self.angle_goal = self.normalize_angle((self.red_object_angle + self.blue_object_angle) / 2)
                self.angle_goal_deg = (self.red_object_angle_deg + self.blue_object_angle_deg) / 2
                self.goal_x = (self.red_x + self.blue_x) / 2
                self.goal_y = (self.red_y + self.blue_y) / 2
                self.goal_distance = math.sqrt(((self.goal_x - self.encoder_odometry[0]) ** 2) + ((self.goal_y - self.encoder_odometry[1]) ** 2))
            elif (self.blue_object_angle - self.red_object_angle) <= math.radians(-180):
                #print("both spheres are on different sides of 0 and correct orientation")
                # if 0 degrees is between the two spheres and blue is on the right side
                self.angle_goal = self.normalize_angle((self.blue_object_angle + self.red_object_angle) / 2 + math.radians(180))
                self.angle_goal_deg = (self.red_object_angle_deg + self.blue_object_angle_deg) / 2 + 180
                self.angle_goal_deg = self.angle_goal_deg % 360
                self.go_around = False
                self.goal_x = (self.red_x + self.blue_x) / 2
                self.goal_y = (self.red_y + self.blue_y) / 2
                self.goal_distance = math.sqrt(((self.goal_x - self.encoder_odometry[0]) ** 2) + ((self.goal_y - self.encoder_odometry[1]) ** 2))
            else:
                #print("wrong orientation")
                self.angle_goal = self.normalize_angle(self.red_object_angle + math.radians(15))
                if self.angle_goal >= math.radians(360):
                    self.angle_goal -= math.radians(360)
                self.go_around = True
                self.goal_distance = 2 * math.sqrt(((self.red_x - self.encoder_odometry[0]) ** 2) + ((self.red_y - self.encoder_odometry[1]) ** 2))
            self.next_state = "move_to_point"
        else:
            #print("Get rotation:  ", self.get_rotation())
            #print("angle goal for turning", self.angle_goal_deg)
            if self.angle_goal_deg - 15 <= self.get_rotation() <= self.angle_goal_deg + 2:
                #print("right angle!")
                self.next_state = "drive_forward"
                self.right_controller.set_desired_pid_speed(0)
                self.left_controller.set_desired_pid_speed(0)
            else:
                # print("Angle Goal: ", self.angle_goal_deg)
                self.left_controller.set_desired_pid_speed(-15)
                self.right_controller.set_desired_pid_speed(15)
                self.next_state = "move_to_point"

    def drive_forward(self):
        # print("Angles:  ", self.angle_goal, self.encoder_odometry[2])
        # print("Goal Distance: ", self.goal_distance)
        if self.previous_state != "drive_forward":
            self.left_controller.reset()
            self.right_controller.reset()
            self.forward_start_time = self.time
            self.start_x = self.encoder_odometry[0]
            self.start_y = self.encoder_odometry[1]
        distance_traveled = math.sqrt((self.encoder_odometry[0] - self.start_x) ** 2 + (self.encoder_odometry[1] - self.start_y) ** 2)
        # print("Distance traveled: ", distance_traveled)
        if distance_traveled >= self.goal_distance:
            if self.go_around:
                self.next_state = "full_scan"
            else:
                self.next_state = "stop"
        else:
            self.left_controller.set_desired_pid_speed(8)
            self.right_controller.set_desired_pid_speed(8)
            self.next_state = "drive_forward"

    def get_rotation(self):
        """Getter method."""
        return abs(self.rotation)

    def stop(self):
        """Default end state."""
        if self.previous_state != "stop":
            self.left_controller.reset()
            self.right_controller.reset()
            self.left_controller.set_desired_pid_speed(0)
            self.right_controller.set_desired_pid_speed(0)
            self.next_state = "stop"
        else:
            self.shutdown = True

    def calculate_motor_power(self):
        """
        Calculate the power for both motor to achieve the desired speed.
        :return: None
        """
        if self.left_goal_speed == 0 and self.right_goal_speed == 0:
            self.left_power = 0
            self.right_power = 0
        else:
            left_error = self.left_wheel_speed - self.left_goal_speed
            print("Left error: ", left_error)
            print("")
            right_error = self.right_wheel_speed - self.right_goal_speed
            print("Right error: ", right_error)
            print("")
            self.left_power -= round(self.left_controller.get_correction(left_error))
            print("left power: ", self.left_power)
            print("")
            self.right_power -= round(self.right_controller.get_correction(right_error))
            print("right power: ", self.right_power)
            print("")

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
                self.red_distance = 14 / object[2]
                red_x_difference = self.camera_center - red_coordinates_x
                red_object_angle = (red_x_difference / self.camera_resolution) * self.camera_field_of_view
                self.red_object_angle_deg = self.get_rotation() + red_object_angle
                red_object_angle = (red_object_angle * math.pi) / 180
                self.red_object_angle = self.normalize_angle(red_object_angle + self.encoder_odometry[2])
                self.red_x = self.red_distance * math.cos(self.red_object_angle)
                self.red_y = self.red_distance * math.sin(self.red_object_angle)
                #print("Red object angle: ", self.red_object_angle_deg)
                #print("robot angle: ", self.get_rotation())
            if object[0] == 'blue sphere' and not self.blue_object_angle:
                self.blue_coordinates_xy = object[1]
                blue_coordinates_x = self.blue_coordinates_xy[0]
                self.blue_distance = 14 / object[2]
                blue_x_difference = self.camera_center - blue_coordinates_x
                blue_object_angle = (blue_x_difference / self.camera_resolution) * self.camera_field_of_view
                self.blue_object_angle_deg = self.get_rotation() + blue_object_angle
                blue_object_angle = (blue_object_angle * math.pi) / 180
                self.blue_object_angle = self.normalize_angle(blue_object_angle + self.encoder_odometry[2])
                self.blue_x = self.blue_distance * math.cos(self.blue_object_angle)
                self.blue_y = self.blue_distance * math.sin(self.blue_object_angle)
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

        # Read wheel encoder values

        self.left_encoder = self.robot.get_left_wheel_encoder()
        self.right_encoder = self.robot.get_right_wheel_encoder()

        self.left_delta = self.left_encoder - self.last_left_encoder
        self.right_delta = self.right_encoder - self.last_right_encoder
        self.rotation_raw = self.robot.get_rotation()
        self.rotation = self.rotation_raw - self.rotation_base
        if self.get_rotation() > 360:
            self.reset_rotation()
        self.detected_objects = self.robot.get_camera_objects()

        # Odometry
        if self.delta_time > 0:
            self.left_wheel_speed = math.radians(self.left_delta) / self.delta_time
            self.right_wheel_speed = math.radians(self.right_delta) / self.delta_time
            self.encoder_odometry[2] = math.radians(self.robot.get_rotation())
            self.encoder_odometry[0] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.cos(
                self.encoder_odometry[2]) * self.delta_time
            self.encoder_odometry[1] += (self.wheel_radius / 2) * (self.left_wheel_speed + self.right_wheel_speed) * math.sin(
                self.encoder_odometry[2]) * self.delta_time
            self.encoder_odometry[2] = self.normalize_angle(self.encoder_odometry[2])
            #print("Odometry:", self.encoder_odometry)

        if self.delta_time != 0:
            self.left_wheel_distance = (self.left_encoder - self.last_left_encoder) / self.delta_time
            self.right_wheel_distance = (self.right_encoder - self.last_right_encoder) / self.delta_time

        self.last_left_encoder = self.left_encoder
        self.last_right_encoder = self.right_encoder
        self.left_controller.update_pid()
        self.right_controller.update_pid()

    def plan(self):
        print("State: ", self.state)
        self.states[self.state]()
        self.previous_state = self.state
        self.state = self.next_state

    def act(self):
        self.calculate_motor_power()
        self.robot.set_left_wheel_speed(self.left_controller.get_pid_output())
        self.robot.set_right_wheel_speed(self.right_controller.get_pid_output())

    def spin(self):
        """The spin loop."""
        while not self.shutdown:
            self.sense()
            self.plan()
            self.act()


def main():
    """Main  entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()
