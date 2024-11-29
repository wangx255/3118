import rclpy
import rclpy.node
from grid_planners_demo.collision_checker import CollisionChecker
from grid_planners_demo.node import Node
from grid_planners_demo.planners.dijkstra import Dijkstra
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import Bool
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener, Buffer
import time


class Navigator(rclpy.node.Node):
    """Manages the incoming start-to-goal query, solves it by requesting a map and sends the path to the controller to follow"""

    def __init__(self):
        super().__init__("navigator_node")

        #! Start and goal nodes for the query
        self.start_ = None
        self.goal_ = None
        self.goal_tolerance_ = (
            self.declare_parameter("goal_tolerance", 0.1)
            .get_parameter_value()
            .double_value
        )  # tolerance of the solution path to the goal

        self.collision_checker_ = None

        # ========================================

        #! Flags to check the status of the robot, the query and the map
        self.is_goal_cancelled_ = False
        self.is_goal_reached_ = False
        self.is_map_loaded_ = False
        self.is_robot_moving_ = False

        self.map_ = None

        self.use_path_smoother_ = (
            self.declare_parameter("use_path_smoother", False)
            .get_parameter_value()
            .bool_value
        )

        self.declare_parameter("max_planning_bounds", [20, 20])
        self.max_planning_bounds_ = self.get_parameter("max_planning_bounds").value

        # ========================================

        #! Information about the robot
        self.robot_position_ = None
        self.robot_orientation_ = None
        self.robot_radius_ = (
            self.declare_parameter("robot_radius", 0.15)
            .get_parameter_value()
            .double_value
        )

        # ========================================
        self.map_topic_ = (
            self.declare_parameter("map_topic", "/map")
            .get_parameter_value()
            .string_value
        )

        self.tf_buffer = Buffer()
        self.transform_listener = TransformListener(self.tf_buffer, self)

        time.sleep(1)

        #! SUBSCRIBERS
        self.start_subscriber = self.create_subscription(
            TFMessage, "/tf", self.robot_pose_callback, 10
        )
        self.goal_subscriber = self.create_subscription(
            PoseStamped, "/goal_pose", self.goal_query_callback, 10
        )

        # ? subscribers to communicate with the controller
        self.robot_moving_subscriber = self.create_subscription(
            Bool, "/robot_is_moving", self.robot_moving_callback, 10
        )

        self.goal_reached_subscriber = self.create_subscription(
            Bool, "/goal_reached", self.goal_reached_callback, 10
        )

        self.map_subscriber = self.create_subscription(
            OccupancyGrid, self.map_topic_, self.map_callback, 10
        )

        # =======================================

        #! PUBLISHERS
        # ? publishers to communicate with the controller
        self.path_publisher = self.create_publisher(Path, "/path", 10)
        self.stop_motion_publisher = self.create_publisher(Bool, "/stop_motion", 10)
        self.goal_publisher = self.create_publisher(PoseStamped, "/goal_controller", 10)

        # ========================================

    def goal_reached_callback(self, goal_reached: Bool):
        """Listens from the controller wether the robot has reached the goal"""
        self.is_goal_reached_ = goal_reached

    def robot_moving_callback(self, robot_moving: Bool):
        """Listens from the controller wether the robot is moving or not"""
        self.is_robot_moving_ = robot_moving

    def robot_pose_callback(self, data: TFMessage):
        """Listen to robot current position"""

        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rclpy.time.Time()
            )
            position = transform.transform.translation
            quaternion = transform.transform.rotation
        except:
            return

        self.robot_position_ = [
            position.x,
            position.y,
            position.z,
        ]
        self.robot_orientation_ = [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]

    def goal_query_callback(self, data: PoseStamped) -> bool:
        """Obtain goal query and ask for the planning callback"""
        if self.is_robot_moving_:
            self.cancel_goal()

        self.goal_ = data.pose
        self.is_goal_cancelled_ = False
        self.is_goal_reached_ = False

        self.get_logger().info("Received new goal")
        self.goal_publisher.publish(data)
        return self.planning_callback()

    def map_callback(self, map: OccupancyGrid):
        "Obtain map used for the planning callback"
        self.map_ = map
        self.is_map_loaded_ = True

    def planning_callback(self):
        """Main planning function where the whole process is carried,
        definition of the start and goal, map and solving of the query"""

        #! 1. Define start and goal states as nodes

        start = Node.from_tf(self.robot_position_, self.robot_orientation_)
        goal = Node.from_pose(self.goal_)

        #! 2. OBTAIN MAP

        if not self.is_map_loaded_:
            return

        #! 3. Define Collision Checker

        collision_checker = CollisionChecker(
            self.map_, self.robot_radius_, self.max_planning_bounds_
        )

        #! 4. Check if Start and Goal states are valid

        if (
            (
                not collision_checker.is_node_free(goal)
                and not collision_checker.is_node_free(start)
            )
            or not collision_checker
            or not start
        ):
            self.get_logger().warning(
                "Goal can't be reached, either start or goal are in collision"
            )
            self.path_publisher.publish(
                self.nodes_to_path_msg([])
            )  # Clearing path from RViz
            return False

        #! 5. Define planner

        planner = Dijkstra()

        planner.set_start(start)  # define start robot position
        planner.set_goal(goal)  # define goal
        planner.set_collision_checker(collision_checker)  # set collision node checker
        planner.set_goal_tolerance(self.goal_tolerance_)  # tolerance to find the goal

        #! 6. Solve the query

        self.get_logger().info("Searching solution path...")
        solved = planner.solve()  # solve the query

        if solved:  # if the query is solved
            path_length = planner.get_path_length()
            self.get_logger().info(f"Path length: {path_length} meters")

            solution_path = planner.get_solution_path()
            self.get_logger().info("Solution path found")
            if self.use_path_smoother_:
                solution_path = self.moving_average(solution_path)
            path_msg = self.nodes_to_path_msg(solution_path)

            #! 7. Send path to controller
            self.path_publisher.publish(path_msg)

            return True

        self.get_logger().warning("No path found")
        self.path_publisher.publish(
            self.nodes_to_path_msg([])
        )  # Clearing path from RViz
        return False

    def moving_average(self, path: list, window: int = 4) -> list:
        """Smoothes the path obtained by finding an average"""

        window_queue = []
        smoothed_path = [path[0]]

        for node in path:
            if len(window_queue) == window:
                smoothed_path.append(sum(window_queue) / window)  # Mean
                window_queue.pop(0)

            window_queue.append(node)
        goal = Node.from_pose(self.goal_)
        return smoothed_path + [goal]

    def nodes_to_path_msg(self, path_nodes: list) -> Path:
        """Transforms the list of path nodes into a Path type of object"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())

        return path

    def cancel_goal(self):
        """Send a cancel to the path follower"""
        self.stop_motion_publisher.publish(Bool(data=True))
        self.is_goal_cancelled_ = True


################################################################
# MAIN DEPLOY
def main(args=None):
    rclpy.init(args=args)
    navigator_node = Navigator()
    try:
        rclpy.spin(navigator_node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        navigator_node.cancel_goal()
    finally:
        rclpy.try_shutdown()

    navigator_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
