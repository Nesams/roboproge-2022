"""OT13 - Planning with PDDL."""
import pyperplan
import pyperplan.planner
import pyperplan.pddl.pddl
import pyperplan.pddl.parser
import PiBot
import math
import os


class TaskSolver:
    """TaskSolver class."""

    def __init__(self):
        """Initialize class."""
        domain = os.path.abspath("spheres_group.pddl")
        parser = pyperplan.pddl.parser.Parser(domain)
        self.domain = parser.parse_domain()
        self.problem = pyperplan.pddl.pddl.Problem("Sphere problem",
                                                   self.domain, {}, [], [])
        for direction in ["dir-left", "dir-right", "dir-down", "dir-up"]:
            self.problem.objects[direction] = self.domain.types["direction"]

    def _generate_directions(self, name, row, col) -> None:
        for dir in [((0, -1), "dir-down"), ((0, 1), "dir-up"),
                    ((-1, 0), "dir-left"), ((1, 0), "dir-right")]:
            parameters = [(name, self.domain.types["location"]),
                          (f"pos-{row + dir[0][0]}-{col + dir[0][1]}",
                           self.domain.types["location"]), (dir[1],
                                                            self.domain.types["location"])]
            predicate = pyperplan.pddl.pddl.Predicate("move-dir", parameters)
            self.problem.initial_state.append(predicate)

    def generate_world(self, origin: tuple, height: int, width: int) -> None:
        """
        Generate the empty rectangular world with empty spaces.

        Arguments:
          origin -- the minimum (x, y) coordinates, i.e., bottom left corner
          width -- the number of cells as width
          height -- the number of cells as height
        """
        for row in range(origin[0], origin[0] + height):
            for col in range(origin[1], origin[1] + width):
                name = f"pos-{row}-{col}"
                self.problem.objects[name] = self.domain.types["location"]
                self.problem.initial_state.append(
                    pyperplan.pddl.pddl.Predicate("is-nongoal", [(name,
                                                                  self.domain.types["location"])]))
                self._generate_directions(name, row, col)
                parameters = [(name, self.domain.types["location"])]
                clear_predicate = pyperplan.pddl.pddl.Predicate("clear", parameters)
                self.problem.initial_state.append(clear_predicate)

    def set_sphere_positions(self, spheres: tuple) -> None:
        """
        Set the sphere constraints in the problem definition.

        Arguments:
          spheres -- a tuple of tuples with sphere information -
                     ((name, type, (location_x, location_y)), ...)
                     name: string,
                     type: string -- either "red-sphere" or "blue-sphere",
                     (location_x: int, location_y: ints)

                     Example: (("red-01", "red-sphere", (1, 1)),
                               ("blue-01", "blue-sphere", (2, 4)))
        """
        for sphere in spheres:
            self.problem.objects[sphere[0]] = self.domain.types[sphere[1]]
            location = f"pos-{sphere[2][0]}-{sphere[2][1]}"
            at_params = [(sphere[0], self.domain.types[sphere[1]]),
                         (location, self.domain.types["location"])]
            at_predicate = pyperplan.pddl.pddl.Predicate("at", at_params)
            self.problem.initial_state.append(at_predicate)
            if sphere[1] == "red-sphere":
                goal = "at-red-goal"
            else:
                goal = "at-blue-goal"
            goal_params = [(sphere[0], ((self.domain.types["sphere"]),))]
            goal_predicate = pyperplan.pddl.pddl.Predicate(goal, goal_params)
            self.problem.goal.append(goal_predicate)
            for i, predicate in enumerate(self.problem.initial_state):
                if (predicate.name == "clear" and
                        predicate.signature[0][0] == location):
                    del self.problem.initial_state[i]
                    break

    def set_goals(self, goals: tuple) -> None:
        """
        Set the goal positions in the problem definition.

        Arguments:
          goals -- a tuple of tuples with goal locations -
                   ((type, location), ...)
                   type: string - either "is-red-goal" or "is-blue-goal"
                   location: tuple with x, y as ints
        """
        for goal in goals:
            parameter = f"pos-{goal[1][0]}-{goal[1][1]}"
            goal_params = [(parameter, self.domain.types["location"])]
            goal_predicate = pyperplan.pddl.pddl.Predicate(goal[0], goal_params)
            self.problem.initial_state.append(goal_predicate)
            for i, predicate in enumerate(self.problem.initial_state):
                if (predicate.name == "is-nongoal" and
                        predicate.signature[0][0] == parameter):
                    del self.problem.initial_state[i]
                    break

    def set_robot_position(self, x: int, y: int) -> None:
        """
        Set the robot starting position in the problem definition.

        Arguments:
          x -- the x cell of the robot starting position.
          y -- the y cell of the robot starting position.
        """
        self.problem.objects["robot-01"] = self.domain.types["robot"]
        location = f"pos-{x}-{y}"
        at_params = [("robot-01", self.domain.types["robot"]),
                     (location, self.domain.types["location"])]
        at_predicate = pyperplan.pddl.pddl.Predicate("at", at_params)
        self.problem.initial_state.append(at_predicate)
        for i, predicate in enumerate(self.problem.initial_state):
            if (predicate.name == "clear" and
                    predicate.signature[0][0] == location):
                del self.problem.initial_state[i]
                break

    def get_domain(self):
        """Return the domain."""
        return self.domain

    def get_problem(self):
        """Return the problem."""
        return self.problem

    def solve(self):
        """Solve the task."""
        task = pyperplan.planner._ground(self.problem)
        heuristic_class = None
        heuristic = None
        if heuristic_class is not None:
            heuristic = heuristic_class(task)
        search = pyperplan.search.breadth_first_search
        return pyperplan.planner._search(task, search, heuristic)


class Robot:
    """Robot class."""

    def __init__(self, initial_angle=0.0):
        """
        Initialize variables.

        Arguments:
          initial_angle -- initial angle in degrees
        """
        self.initial_angle = initial_angle
        self.robot = PiBot.PiBot()
        self.cell_size = 0.25
        self.task_solver = TaskSolver()

        self.front_middle_laser = 0
        self.camera_objects = {}
        self.objects = {}

        self.start_rotation = self.robot.get_rotation()
        self.current_rotation = 0

        self.cells = []
        self.cells_tuple = []

        self.min_x = 0
        self.min_y = 0
        self.sphere = 0
        self.sphere_tuple = ()

        self.bottom_left = [0, 0]
        self.top_right = [0, 0]
        self.increment = 0

    def set_robot(self, robot: PiBot.PiBot()) -> None:
        """Set the robot reference."""
        self.robot = robot

    def get_pddl_data(self):
        """Return the problem description for feedback visualizer."""
        return self.task_solver.problem.initial_state

    def sense(self):
        """Sense according the SPA architecture."""
        """
        Acquire readings from sensors.
        """
        if self.robot.get_rotation() != 0:
            self.current_rotation = self.robot.get_rotation() - self.start_rotation
        self.front_middle_laser = self.robot.get_front_middle_laser()
        self.camera_objects = self.robot.get_camera_objects()
        if self.camera_objects is not None:
            if self.front_middle_laser < 2 and len(self.camera_objects) > 0:
                for o in self.camera_objects:
                    if o[0] not in self.objects.values():
                        self.objects[((self.front_middle_laser * math.cos(self.current_rotation)) / self.cell_size,
                                      (self.front_middle_laser * math.sin(self.current_rotation)) / self.cell_size)] = \
                            (self.increment, self.camera_objects[0], ((self.front_middle_laser
                                                                       * math.cos(self.current_rotation)) / self.cell_size,
                                                                      (self.front_middle_laser * math.sin(
                                                                          self.current_rotation)) / self.cell_size))
                        self.increment += 1

    def create_goals(self, world):
        """Create the goals.

        Arguments:
          world -- a dictionary with keys as coordinates (x, y),
                   values are not used in this method.
                   e.g., {(-1, 1): ('blue-1', 'blue-sphere', (-1, 1)),
                          (0, -1): ('red-2', 'red-sphere', (0, -1))}
        """
        i = 0
        goals = []
        for row in range(math.ceil(math.sqrt(len(world)))):
            for col in range(math.ceil(math.sqrt(len(world)))):
                goals.append(("is-red-goal" if i % 2 == 0 else "is-blue-goal",
                              (row, col)))
                i += 1
                if i >= len(world):
                    return goals
        return goals

    def map_surroundings(self):
        """Map the surroundings."""
        for o in self.objects.keys():
            if o[0] + o[1] < self.bottom_left[0] + self.bottom_left[1]:
                self.bottom_left = [o[0], o[1]]
            elif o[0] + o[1] > self.top_right[0] + self.top_right[1]:
                self.top_right = [o[0], o[1]]
        sphere_tuple = []
        for o in self.objects.values():
            sphere_tuple.append(o)
        self.sphere_tuple = sphere_tuple

    def solve(self):
        """
        Solve the problem.

        Use sensors to map the surroundings, generate a grid map
        and then use TaskSolver to solve the task.
        """
        task_solver = TaskSolver()
        self.map_surroundings()
        task_solver.generate_world((self.bottom_left[0], self.bottom_left[1]), 15, 15)
        task_solver.set_sphere_positions(self.sphere_tuple)
        task_solver.set_goals(tuple(self.create_goals(self.objects)))
        task_solver.set_robot_position(0, 0)


def main():
    """Main entry point."""
    robot = Robot(initial_angle=0)
    solution = robot.solve()
    if solution is None:
        print("No solution could be found")
    else:
        print("Plan length: %s" % len(solution))
        print(solution)


if __name__ == "__main__":
    main()
