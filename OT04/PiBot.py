"""
To use this helper file, please add it to the same directory as your OT04.py file along with the data files (the files that has the get_data() function in it).

The directory should have the following files in it:
.
├── OT04.py
├── PiBot.py
└── leaning_right.py


Add the following code to your OT04.py:

def test():
    robot = Robot()
    import leaning_right # or any other data file
    data = leaning_right.get_data()
    robot.robot.load_data_profile(data)
    for i in range(999):
        print(f"left_encoder = {robot.robot.get_rightmost_line_sensor()}")
        robot.robot.sleep(0.05)

if __name__ == "__main__":
    test()
"""
import time
class PiBot:
    def __init__(self):
        self.encoders = []
        self.time = 0.0
        self.WHEEL_DIAMETER = 0.03
        self.idx = 0
        self.data = None

    def load_data_profile(self, data):
        self.data = data

    def get_leftmost_line_sensor(self):
        if self.data is not None:
            return self.data[self.idx][1][2]

    def get_second_line_sensor_from_left(self):
        if self.data is not None:
            return self.data[self.idx][1][1]

    def get_third_line_sensor_from_left(self):
        if self.data is not None:
            return self.data[self.idx][1][0]

    def get_third_line_sensor_from_right(self):
        if self.data is not None:
            return self.data[self.idx][1][3]

    def get_second_line_sensor_from_right(self):
        if self.data is not None:
            return self.data[self.idx][1][4]

    def get_rightmost_line_sensor(self):
        if self.data is not None:
            return self.data[self.idx][1][5]

    def sleep(self, seconds):
        self.time += seconds
        for i in range(self.idx, len(self.data)):
            if self.time >= self.data[i][0]:
                self.idx = i
            elif self.time < self.data[i][0]:
                break

    def get_time(self):
        return self.time

