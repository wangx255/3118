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
        rclpy.logging.get_logger("A* Planner").info(
            f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds"
        )
        return result

    return timeit_wrapper


class AStar:
    """A* algorithm implementation as a planner to solve start-to-goal queries"""

    def __init__(self):
        self.start = None
        self.goal = None
        self.collision_checker = None
        self.tolerance = None
        self.solution_path = None

    # SETTERS

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

    # GETTERS

    def get_solution_path(self):
        """Obtain stored solution path"""
        return self.solution_path

    def get_path_length(self):
        """Obtain the length of the solution path"""
        if not self.solution_path:
            return 0.0
        distance = 0.0
        for i in range(len(self.solution_path) - 1):
            dx = self.solution_path[i+1].x - self.solution_path[i].x
            dy = self.solution_path[i+1].y - self.solution_path[i].y
            distance += math.sqrt(dx * dx + dy * dy)
        return distance

    def _node_key(self, node):
        """Generate a key for a node by rounding coordinates to 4 decimal places"""
        return (round(node.x, 4), round(node.y, 4))

    @timeit
    def solve(self) -> bool:
        """Function to find path from start to goal using A* algorithm"""

        # CHECK IF ALL THE REQUIRED ATTRIBUTES HAVE BEEN SET
        if not self.collision_checker:
            rclpy.logging.get_logger("A* Planner").warning(
                "Collision checker has not been set on the planner"
            )
            return False
        if not self.start:
            rclpy.logging.get_logger("A* Planner").warning(
                "Start position has not been set on the planner"
            )
            return False
        if not self.goal:
            rclpy.logging.get_logger("A* Planner").warning(
                "Goal position has not been set on the planner"
            )
            return False

        # START SOLVING THE QUERY
        open_list = []
        start_key = self._node_key(self.start)
        heapq.heappush(open_list, (0.0, self.start))
        open_set = {start_key}

        closed_set = set()

        g_score = {start_key: 0.0}

        while open_list:
            current_f, current_node = heapq.heappop(open_list)
            current_key = self._node_key(current_node)

            if current_key not in open_set:
                # This node has already been processed
                continue

            open_set.remove(current_key)

            # Check if current node is within tolerance to goal
            if current_node.calculate_distance(self.goal) <= self.tolerance:
                self.solution_path = current_node.backtrack_path()
                rclpy.logging.get_logger("A* Planner").info("Path found successfully.")
                return True

            closed_set.add(current_key)

            # Generate neighbors
            neighbors = current_node.generate_neighbors(self.collision_checker.map_resolution)
            for neighbor in neighbors:
                neighbor_key = self._node_key(neighbor)

                if neighbor_key in closed_set:
                    continue

                if not self.collision_checker.is_node_free(neighbor):
                    continue

                tentative_g = g_score[current_key] + current_node.calculate_distance(neighbor)

                if neighbor_key not in g_score or tentative_g < g_score[neighbor_key]:
                    g_score[neighbor_key] = tentative_g
                    neighbor.parent = current_node
                    # Assuming heuristic is Euclidean distance
                    neighbor.h = neighbor.calculate_distance(self.goal)
                    neighbor.f = tentative_g + neighbor.h

                    if neighbor_key not in open_set:
                        heapq.heappush(open_list, (neighbor.f, neighbor))
                        open_set.add(neighbor_key)
                    else:
                        # Re-add neighbor with updated f-score
                        heapq.heappush(open_list, (neighbor.f, neighbor))

        # If open list is exhausted and no path found
        rclpy.logging.get_logger("A* Planner").warning("No path found.")
        return False


