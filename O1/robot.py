"""Object Detection."""
import math
import statistics
import PiBot

class Robot:
    """Robot class."""

    def __init__(self):
        """Class constructor."""
        self.started_driving = 0
        self.robot = PiBot.PiBot()
        self.shutdown = False
        self.objects = []
        self.front_middle_laser = None
        self.filter_list = []
        self.front_middle_laser = 0
        self.left_encoder = 0
        self.right_encoder = 0
        self.wheel_diameter = self.robot.WHEEL_DIAMETER
        self.axis_length = self.robot.AXIS_LENGTH
        self.wheel_circumference = self.robot.WHEEL_DIAMETER * math.pi
        self.object_start = 5
        self.object_start_and_end = []
        self.min_object_distance = 100
        self.circle = self.axis_length * math.pi
        self.last_laser_reading = 0.5
        self.object_detected = False
        self.wheel_distance = 0
        self.left_wheel_speed = 0
        self.right_wheel_speed = 0
        self.object_distance = 0
        self.time = 0
        self.drive_straight = False

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set Robot reference."""
        self.robot = robot

    def get_objects(self) -> list:
        """
        Return the list with the detected objects so far.

        (i.e., add new objects to the list as you detect them).

        Returns:
          The list with detected object angles, the angles are in
          degrees [0..360), 0 degrees being the start angle and following
          the right-hand rule (e.g., turning left 90 degrees is 90, turning
          right 90 degrees is 270 degrees).
        """
        if len(self.filter_list) == 5:
            if self.get_front_middle_laser() <= 0.55:
                self.object_detected = True
                self.object_distance = self.get_front_middle_laser()
                if self.object_distance < self.min_object_distance:
                    self.min_object_distance = self.object_distance
                self.object_start_and_end.append(self.get_current_angle())
            elif self.get_front_middle_laser() >= 0.6 and self.object_detected:
                print(len(self.object_start_and_end), '===================')
                object_angle = (self.object_start_and_end[0] + self.object_start_and_end[-1]) / 2

                self.object_detected = False
                self.objects.append(object_angle)
                #if 310 <= self.object_start_and_end[-1] <= 360 and 0 <= self.object_start_and_end[0]:
                    #self.objects[-1] -= 180
                self.object_start_and_end.clear()
        return self.objects

    def get_current_angle(self):
        """"jshdhbd."""
        self.wheel_distance = self.right_encoder / 360 * self.wheel_circumference
        current_angle = self.wheel_distance * 360 / self.circle
        return current_angle % 360

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
        """Sense method according to the SPA architecture."""
        self.right_encoder = self.robot.get_right_wheel_encoder()
        self.left_encoder = self.robot.get_left_wheel_encoder()
        self.front_middle_laser = self.robot.get_front_middle_laser()
        self.time = self.robot.get_time()
        if self.front_middle_laser is not None:
            self.filter_list.insert(0, self.front_middle_laser)
            self.filter_list = self.filter_list[:5]

    def plan(self):
        if not self.objects:
            self.get_objects()
            self.left_wheel_speed = -8
            self.right_wheel_speed = 8
        else:
            print(self.get_current_angle(), self.objects, abs(self.get_current_angle() - self.objects[0]) < 1)
            if abs(self.get_current_angle() - self.objects[0]) < 1 or self.drive_straight:
                self.drive_straight = True
                if self.started_driving == 0:
                    self.started_driving = self.time + self.min_object_distance * 9
                if self.started_driving < self.time:
                    self.left_wheel_speed = 0
                    self.right_wheel_speed = 0
                    self.shutdown = True
                else:
                    self.left_wheel_speed = 10
                    self.right_wheel_speed = 10
            elif not self.drive_straight:
                self.left_wheel_speed = -8
                self.right_wheel_speed = 8

    def act(self):
        self.robot.set_left_wheel_speed(self.left_wheel_speed)
        self.robot.set_right_wheel_speed(self.right_wheel_speed)

    def spin(self):
        """The main loop."""
        while not self.shutdown:
            self.sense()
            self.plan()
            self.act()
            self.robot.sleep(0.05)

def main():
    """The  main entry point."""
    robot = Robot()
    robot.spin()


if __name__ == "__main__":
    main()
