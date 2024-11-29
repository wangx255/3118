#!/usr/bin/env python3

import time
import rclpy
import rclpy.node
from collision_checker import CollisionChecker
from node import Node
from planners.rrt import RRT
from geometry_msgs.msg import Pose, PoseStamped, Point
from nav_msgs.msg import OccupancyGrid, Path, Odometry
from std_msgs.msg import Bool
from visualization_msgs.msg import Marker
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformListener, Buffer


class Navigator(rclpy.node.Node):
    """Manages the incoming start-to-goal query, solves it by requesting a map and sends the path to the controller to follow"""

    def __init__(self):
        super().__init__("navigator_node")

        #! Start and goal nodes for the query
        self.start = None
        self.goal = None

        self.collision_checker = None

        # ========================================

        #! Flags to check the status of the robot, the query and the map
        self.is_goal_cancelled = False
        self.is_goal_reached = False
        self.is_map_loaded = False
        self.is_robot_moving = False

        self.step_size = (
            self.declare_parameter("step_size", 0.001)
            .get_parameter_value()
            .double_value
        )
        self.expansion_size = (
            self.declare_parameter("expansion_size", 0.5)
            .get_parameter_value()
            .double_value
        )
        self.solve_time = (
            self.declare_parameter("solve_time", 5.0).get_parameter_value().double_value
        )
        self.bias = (
            self.declare_parameter("bias", 0.5).get_parameter_value().double_value
        )

        self.goal_tolerance = (
            self.declare_parameter("goal_tolerance", 0.1)
            .get_parameter_value()
            .double_value
        )  # tolerance of the solution path to the goal

        # ? flag to decide use of path smoother
        self.use_path_smoother = (
            self.declare_parameter("use_path_smoother", False)
            .get_parameter_value()
            .bool_value
        )

        self.declare_parameter("max_planning_bounds", [20, 20])
        self.max_planning_bounds = self.get_parameter("max_planning_bounds").value

        # ========================================

        #! Information about the robot
        self.robot_position = None
        self.robot_orientation = None
        self.robot_radius = (
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
        self.tree_publisher = self.create_publisher(Marker, "/rrt", 10)

        # ========================================

    def goal_reached_callback(self, goal_reached: Bool):
        """Listens from the controller whether the robot has reached the goal"""
        self.is_goal_reached = goal_reached.data

    def robot_moving_callback(self, robot_moving: Bool):
        """Listens from the controller whether the robot is moving or not"""
        self.is_robot_moving = robot_moving.data

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

        self.robot_position = [
            position.x,
            position.y,
            position.z,
        ]
        self.robot_orientation = [
            quaternion.x,
            quaternion.y,
            quaternion.z,
            quaternion.w,
        ]

    def goal_query_callback(self, data: PoseStamped) -> bool:
        """Obtain goal query and ask for the planning callback"""
        if self.is_robot_moving:
            self.cancel_goal()

        self.goal = data.pose
        self.is_goal_cancelled = False
        self.is_goal_reached = False

        self.get_logger().info("Received new goal")
        self.goal_publisher.publish(data)
        return self.planning_callback()

    def map_callback(self, map: OccupancyGrid):
        "Obtain map used for the planning callback"
        self.map_ = map
        self.is_map_loaded = True

    def planning_callback(self):
        """Main planning function where the whole process is carried,
        definition of the start and goal, map and solving of the query"""

        #! 1. Define start and goal states as nodes
        start = Node.from_tf(self.robot_position, self.robot_orientation)
        goal = Node.from_pose(self.goal)

        #! 2. OBTAIN MAP
        if not self.is_map_loaded:
            return

        #! 3. Define Collision Checker

        self.collision_checker = CollisionChecker(
            self.map_, self.robot_radius, self.max_planning_bounds
        )

        #! 4. Check if Start and Goal states are valid

        if (
            (
                not self.collision_checker.is_node_free(goal)
                and not self.collision_checker.is_node_free(start)
            )
            or not self.collision_checker
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

        planner = RRT()

        planner.set_start(start)  # define start robot position
        planner.set_goal(goal)  # define goal
        planner.set_collision_checker(
            self.collision_checker
        )  # set collision node checker
        planner.set_goal_tolerance(self.goal_tolerance)  # tolerance to find the goal
        planner.set_expansion_size(self.expansion_size)
        planner.set_step_size(self.step_size)
        planner.set_bias(self.bias)

        #! 6. Solve the query

        self.get_logger().info("Searching solution path...")
        solved = planner.solve(self.solve_time)  # solve the query

        if solved:  # if the query is solved
            path_length = planner.get_path_length()
            self.get_logger().info(f"Path length: {path_length} meters")

            solution_path = planner.get_solution_path()
            self.get_logger().info("Solution path found")
            if self.use_path_smoother:
                solution_path = self.moving_average(solution_path)
            path_msg = self.nodes_to_path_msg(solution_path)

            # publish tree
            rrt = planner.get_tree()
            self.publish_tree(rrt)

            #! 7. Send path to controller
            self.path_publisher.publish(path_msg)

            return True

        # publish tree
        rrt = planner.get_tree()
        self.publish_tree(rrt)

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
        goal = Node.from_pose(self.goal)
        return smoothed_path + [goal]

    def nodes_to_path_msg(self, path_nodes: list) -> Path:
        """Transforms the list of path nodes into a Path type of object"""
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.get_clock().now().to_msg()

        for node in path_nodes:
            path.poses.append(node.to_pose_stamped())

        return path

    def publish_tree(self, tree):
        visual_rrt = Marker()
        visual_rrt.header.frame_id = "map"
        visual_rrt.header.stamp = self.get_clock().now().to_msg()
        visual_rrt.action = Marker.ADD

        visual_rrt.type = Marker.LINE_LIST

        visual_rrt.pose.orientation.w = 1.0

        visual_rrt.id = 0

        visual_rrt.scale.x = 0.03

        visual_rrt.color.b = 1.0
        visual_rrt.color.a = 1.0

        for branch in tree:
            node = tree[branch]

            while node.parent:
                p = Point()
                p.x = node.x
                p.y = node.y
                p.z = 0.1

                visual_rrt.points.append(p)

                p = Point()

                p.x = node.parent.x
                p.y = node.parent.y
                p.z = 0.1
                visual_rrt.points.append(p)
                # print(p)
                node = node.parent
        self.tree_publisher.publish(visual_rrt)

    def send_goal(self, pose: Pose) -> bool:
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = pose

        return self.goal_query_callback(goal)

    def cancel_goal(self):
        self.stop_motion_publisher.publish(Bool(data=True))
        self.is_goal_cancelled = True


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
