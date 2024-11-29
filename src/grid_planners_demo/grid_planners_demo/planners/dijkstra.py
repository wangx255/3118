import heapq
import math
import time
from functools import wraps
import rclpy


# Time measurement function
# =============================
def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        rclpy.logging.get_logger("Dijkstra planner").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


# =============================


class Dijkstra:
    """Dijsktra algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        self.start = None
        self.goal = None

        # grid searching variables
        self.open_list = []
        self.closed_list = []
        self.g = {}
        self.nodes = {}

        self.collision_checker = None
        self.tolerance = None

        self.solution_path = None

    # SETTERS
    # =============================

    def set_start(self, start):
        """Set start position of the robot"""
        self.start = start

    def set_goal(self, goal):
        """Set goal of the start to goal query"""
        self.goal = goal

    def set_collision_checker(self, collision_checker):
        """Set collision checker to detect collision free nodes"""
        self.collision_checker = collision_checker

    def set_goal_tolerance(self, tolerance):
        """Goal tolerance to check when the query has been solved"""
        self.tolerance = tolerance

    # =============================

    # GETTERS
    # =============================

    def get_solution_path(self):
        """Obtain stored solution path"""
        return self.solution_path

    def get_path_length(self):
        """Obtain the length of the solution path"""
        distance = 0

        for i in range(0, len(self.solution_path) - 2):
            distance += math.sqrt(
                math.pow(self.solution_path[i].x - self.solution_path[i + 1].x, 2)
                + math.pow(self.solution_path[i].y - self.solution_path[i + 1].y, 2)
            )

        return distance

    # =============================

    @timeit
    def solve(self) -> list:
        """Function to find path from start to goal using Dijkstra algorithm"""

        # CHECK IF ALL THE REQUIRED ATTRIBUTES HAVE BEEN SET
        if not self.collision_checker:
            rclpy.logging.get_logger("Dijkstra planner").warning(
                "Collision checker has not been set on the planner"
            )
            return False
        elif not self.start:
            rclpy.logging.get_logger("Dijkstra planner").warning(
                "Start position has not been set on the planner"
            )
            return False
        elif not self.goal:
            rclpy.logging.get_logger("Dijkstra planner").warning(
                "Goal position has not been set on the planner"
            )
            return False
        # =============================

        # START SOLVING THE QUERY

        # initialise first nodes, start and goal
        self.g[(self.start.x, self.start.y)] = 0
        self.g[(self.goal.x, self.goal.y)] = math.inf

        # add nodes to priority queue
        heapq.heappush(self.open_list, (0.0, (self.start.x, self.start.y)))
        self.nodes[(self.start.x, self.start.y)] = self.start

        # iterate over the priority queue
        while len(self.open_list) > 0:
            temp_node = heapq.heappop(self.open_list)[1]
            current_node = self.nodes[temp_node]

            # search for neighbors of the node
            for neighbor in current_node.generate_neighbors(
                self.collision_checker.map_resolution
            ):
                if self.collision_checker.is_node_free(neighbor):
                    if (
                        neighbor.calculate_distance(self.goal) < self.tolerance
                    ):  # Check if goal has been achieved
                        neighbor.parent = current_node
                        self.solution_path = (
                            neighbor.backtrack_path()
                        )  # obtain path by backtracking neighbor parents

                        if len(self.solution_path) > 0:
                            return True
                        else:
                            return False

                    # if neighbor is new, it is stored with an inf value
                    if (neighbor.x, neighbor.y) not in self.g:
                        self.g[(neighbor.x, neighbor.y)] = math.inf

                    # define new cost for node
                    new_cost = current_node.g + neighbor.calculate_distance(
                        current_node
                    )

                    # check if new cost is less than neighbor node cost
                    if new_cost < self.g[(neighbor.x, neighbor.y)]:
                        # if cost is lower assign new parent
                        self.g[(neighbor.x, neighbor.y)] = new_cost
                        neighbor.g = new_cost
                        neighbor.parent = current_node

                        # add the neighbor node to the priority queue
                        heapq.heappush(
                            self.open_list, (neighbor.g, (neighbor.x, neighbor.y))
                        )
                        self.nodes[(neighbor.x, neighbor.y)] = neighbor
            # add searched node to the closed_list
            self.closed_list.append(current_node)

        return False
