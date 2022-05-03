"""OT12 - Potential Field Gradient Descent."""
import PiBot
import math


class Robot:
    """The robot class."""

    def __init__(self, attraction_threshold: float = 0.2,
                 attraction_coefficient: float = 2.0,
                 repulsion_threshold: float = 0.4,
                 repulsion_coefficient: float = 1.0):
        """Initialize variables."""
        self.robot = PiBot.PiBot()
        self.obstacles = []
        self.attraction_threshold = attraction_threshold
        self.attraction_coefficient = attraction_coefficient
        self.repulsion_threshold = repulsion_threshold
        self.repulsion_coefficient = repulsion_coefficient

    @staticmethod
    def distance(first: float, second: float) -> float:
        """
        Calculate the Euclidean distance between two points.

        Returns:
          The distance between two points.
        """
        return math.sqrt((first[0] - second[0])**2 + (first[1] - second[1])**2)

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Se the API reference."""
        self.robot = robot

    def set_obstacles(self, obstacles: tuple) -> None:
        """
        Set the obstacles.

        Set the obstacles (i.e., repulsive potential sources) for
        potential field planning.

        Arguments:
          obstacles -- the tuple of 2-tuples defining the obstacles.
                       E.g., ((1, 1), (2, 2), ...)
        """
        self.obstacles = []
        for obstacle in obstacles:
            self.obstacles.append((obstacle[0], obstacle[1], 1))

    def compute_attractor_gradient(self, point: tuple, goal: tuple) -> tuple:
        """
        Compute the attraction gradient (the vector pointing toward the goal).

        Args:
          point -- the point where the gradient is calculated at.
                   Tuple with x, y coordinates
          goal -- the goal as tuple with x, y coordinates
        Returns:
          Returns gradient vector
        """
        d = Robot.distance(point, goal)
        if d <= self.attraction_threshold:
            u_att_x = self.attraction_coefficient * (point[0] - goal[0])
            u_att_y = self.attraction_coefficient * (point[1] - goal[1])
        else:
            u_att_x = (self.attraction_threshold *
                       self.attraction_coefficient * (point[0] - goal[0])) / d
            u_att_y = (self.attraction_threshold *
                       self.attraction_coefficient * (point[1] - goal[1])) / d
        return (u_att_x, u_att_y)

    def compute_repulsion_gradient(self, point: tuple,
                                   obstacles: tuple) -> tuple:
        """
        Compute the repulsion gradient.

        Compute the repulsion gradient (the combined vector pointing away
        from the obstacles).

        Args:
          point -- the point where the gradient is calculated at.
                   Tuple with x, y coordinates
          obstacles -- the obstacles as a tuple with tuples with x, y
                       coordinates
        Returns:
          Returns gradient vector
        """
        u_rep_x = 0
        u_rep_y = 0
        for obstacle in obstacles:
            d = Robot.distance(point, obstacle)
            if d <= self.repulsion_threshold:
                deg = math.atan2(obstacle[1] - point[1],
                                 obstacle[0] - point[0])
                x = point[0] + d * math.cos(deg)
                y = point[1] + d * math.sin(deg)
                u_rep_x += self.repulsion_coefficient \
                    * ((1 / self.repulsion_threshold) - (1 / d)) \
                    * (1 / d)**2 * ((point[0] - x) / d)
                u_rep_y += self.repulsion_coefficient \
                    * ((1 / self.repulsion_threshold) - (1 / d)) \
                    * (1 / d)**2 * ((point[1] - y) / d)
        return (u_rep_x, u_rep_y)

    def calculate_plan(self, start: tuple, goal: tuple, step_size: float,
                       goal_tolerance: float = 0.1) -> list:
        """
        Calculate the plan from start to goal or multiple goals via waypoints.

        Arguments:
          start -- start coordinates (x, y) as floats
          goal -- tuple with goal coordinates (x, y) as floats
          step_size -- the scalar value for each step in the plan (in meters)
          goal_tolerance -- the goal tolerance (acceptable +/-
                            from goal to terminate the algorithm)

        Returns:
          Trajectory to reach the goal as a list of coordinates
          (e.g., [(0, 0.5), (0, 1), (0, 1.5), (0, 2)])
        """
        current_position = start
        trajectory = [start]
        distance_from_object = math.sqrt((current_position[0] - goal[0]) ** 2 + (current_position[1] - goal[1]) ** 2)
        while distance_from_object > goal_tolerance:
            gradient_X = self.compute_attractor_gradient(current_position, goal)[0] + self.compute_repulsion_gradient(current_position, tuple(self.obstacles))[0]
            gradient_Y = self.compute_attractor_gradient(current_position, goal)[1] + self.compute_repulsion_gradient(current_position, tuple(self.obstacles))[1]
            gradient = [gradient_X, gradient_Y]
            position_X = current_position[0] - gradient_X * step_size
            position_Y = current_position[1] - gradient_Y * step_size
            new_position = [position_X, position_Y]
            current_position = tuple(new_position)
            trajectory.append(current_position)
            distance_from_object = math.sqrt((position_X - goal[0]) ** 2 + (position_Y - goal[1]) ** 2)
        return trajectory


def test():
    """Testing function."""
    robot = Robot(attraction_coefficient=2, repulsion_coefficient=0.5,
                  repulsion_threshold=2)
    import straight  # or any other data file
    data = straight.get_data()
    robot.set_obstacles(data[1])
    plan = robot.calculate_plan((0, 0), data[0], 0.1, 0.1)
    # plot(plan)
    print(f"{list(map(lambda x: (round(x[0], 3), round(x[1], 3)), plan))}")


def plot(plan: tuple):
    """Plot with matplotlib."""
    import matplotlib.pyplot
    zipped = list(zip(*plan))
    matplotlib.pyplot.plot(zipped[0], zipped[1])
    matplotlib.pyplot.axis([0, max(max(zipped[0]), max(zipped[1])), 0,
                           max(max(zipped[0]), max(zipped[1]))])
    matplotlib.pyplot.savefig("fig.png")


def main():
    """The main entry point."""
    robot = Robot()
    robot.set_obstacles(((0.2, 3), (-0.3, 4)))
    plan = robot.calculate_plan((0, 0), (0, 6), 0.15)
    print(f"{list(map(lambda x: (round(x[0], 3), round(x[1], 3)), plan))}")


if __name__ == "__main__":
    # main()
    test()
