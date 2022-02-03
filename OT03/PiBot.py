"""
To use this helper file, please add it to the same directory as your OT03.py file along with the data files (the files that has the get_data() function in it).

The directory should have the following files in it:
.
├── OT03.py
├── PiBot.py
└── constant_slow.py


Add the following code to your OT02.py:

def test():
    robot = Robot()
    import constant_slow # or any other data file
    data = constant_slow.get_data()
    robot.robot.load_velocity_profile(data)
    for i in range(len(data)):
        print(f"left_encoder = {robot.robot.get_left_wheel_encoder()}")
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
        self.left_wheel_encoder = 0
        self.right_wheel_encoder = 0
        self.idx = 0
        self.data = None

    def load_velocity_profile(self, data):
        self.data = data

    def update_encoders(self):
        """
        Find the correct data list index and set the current encoder values to the data contained at that index.
        """
        if self.data is None:
            return
        idx = self.idx
        for i, data in enumerate(self.data):
            if data[0] > self.time:
               break
            idx = i
        self.idx = idx
        self.left_wheel_encoder = self.data[idx][1][0]
        self.right_wheel_encoder = self.data[idx][1][1]

    def get_right_wheel_encoder(self):
        self.update_encoders()
        return self.right_wheel_encoder

    def get_left_wheel_encoder(self):
        self.update_encoders()
        return self.left_wheel_encoder

    def sleep(self, seconds):
        self.time += seconds

    def get_time(self):
        return self.time
