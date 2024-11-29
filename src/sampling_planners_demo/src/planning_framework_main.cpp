#include <iostream>
#include <vector>
#include <memory>
#include <cmath>

#include <boost/bind.hpp>

// OMPL
#include <ompl/base/MotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/planners/rrt/RRT.h>
#include <ompl/geometric/planners/prm/PRMstar.h>
#include <ompl/geometric/planners/prm/PRM.h>
#include <ompl/geometric/SimpleSetup.h>
#include <ompl/config.h>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/timer.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/service.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// ROS2 messages
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <nav_msgs/msg/path.hpp>

// Planner
#include <state_validity_checker_octomap_fcl_R2.hpp>

namespace ob = ompl::base;
namespace og = ompl::geometric;

//!  PlannFramework class.
/*!
 * Planning Framework.
 * Setup a sampling-based planner for computation of collision-free paths.
 * C-Space: R2
 * Workspace is represented with Octomaps
 */
class PlannFramework : public rclcpp::Node
{
public:
    //! Constructor
    PlannFramework();
    //! Planner setup
    void run();
    //! Periodic callback to solve the query.
    void planningCallback();
    //! Callback for getting current vehicle odometry
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg);
    //! Callback for getting the 2D navigation goal
    void queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr nav_goal_msg);
    //! Procedure to visualize the resulting path
    void visualizeRRT(og::PathGeometric &geopath);

private:
    // ROS2
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr solution_path_rviz_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr solution_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stop_motion_pub_;

    // ROS TF
    tf2::Transform last_robot_pose_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // OMPL planner
    og::SimpleSetupPtr simple_setup_;
    double timer_period_, solving_time_, goal_tolerance_, yaw_goal_tolerance_, robot_base_radius_;
    bool odom_available_, goal_available_, visualize_tree_;
    std::vector<double> planning_bounds_x_, planning_bounds_y_, start_state_, goal_map_frame_, goal_odom_frame_;
    double goal_radius_;
    std::string planner_name_, odometry_topic_, query_goal_topic_, solution_path_topic_, world_frame_, octomap_service_;
    std::vector<const ob::State *> solution_path_states_;

    rclcpp::Client<GetOctomap>::SharedPtr octomap_client_;
};

//!  Constructor.
/*!
 * Load planner parameters from configuration file.
 * Publishers to visualize the resulting path.
 */
PlannFramework::PlannFramework()
    : Node("plann_framework")
{
    //=======================================================================
    // ! Get parameters
    //=======================================================================
    planning_bounds_x_.resize(2);
    planning_bounds_y_.resize(2);
    start_state_.resize(2);
    goal_map_frame_.resize(3);
    goal_odom_frame_.resize(3);

    this->declare_parameter<std::string>("world_frame", "map");
    this->declare_parameter<std::vector<double>>("planning_bounds_x", {0.0, 10.0});
    this->declare_parameter<std::vector<double>>("planning_bounds_y", {0.0, 10.0});
    this->declare_parameter<std::vector<double>>("start_state", {0.0, 0.0});
    this->declare_parameter<std::vector<double>>("goal_state", {5.0, 5.0, 0.0});
    this->declare_parameter<double>("timer_period", 1.0);
    this->declare_parameter<double>("solving_time", 1.0);
    this->declare_parameter<std::string>("planner_name", "RRTstar");
    this->declare_parameter<std::string>("odometry_topic", "/odom");
    this->declare_parameter<std::string>("query_goal_topic", "/move_base_simple/goal");
    this->declare_parameter<std::string>("solution_path_topic", "/path");
    this->declare_parameter<double>("goal_tolerance", 0.2);
    this->declare_parameter<double>("yaw_goal_tolerance", 0.1);
    this->declare_parameter<bool>("visualize_tree", false);
    this->declare_parameter<double>("robot_base_radius", 0.5);
    this->declare_parameter<std::string>("octomap_service", "/octomap_full");

    this->get_parameter("world_frame", world_frame_);
    this->get_parameter("planning_bounds_x", planning_bounds_x_);
    this->get_parameter("planning_bounds_y", planning_bounds_y_);
    this->get_parameter("start_state", start_state_);
    this->get_parameter("goal_state", goal_map_frame_);
    this->get_parameter("timer_period", timer_period_);
    this->get_parameter("solving_time", solving_time_);
    this->get_parameter("planner_name", planner_name_);
    this->get_parameter("odometry_topic", odometry_topic_);
    this->get_parameter("query_goal_topic", query_goal_topic_);
    this->get_parameter("solution_path_topic", solution_path_topic_);
    this->get_parameter("goal_tolerance", goal_tolerance_);
    this->get_parameter("yaw_goal_tolerance", yaw_goal_tolerance_);
    this->get_parameter("visualize_tree", visualize_tree_);
    this->get_parameter("robot_base_radius", robot_base_radius_);
    this->get_parameter("octomap_service", octomap_service_);

    goal_radius_ = goal_tolerance_;
    goal_available_ = false;

    //=======================================================================
    // ! Subscribers
    //=======================================================================
    // Odometry data
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(odometry_topic_, 10, std::bind(&PlannFramework::odomCallback, this, std::placeholders::_1));
    odom_available_ = false;

    // 2D Nav Goal
    nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(query_goal_topic_, 10, std::bind(&PlannFramework::queryGoalCallback, this, std::placeholders::_1));

    //=======================================================================
    // ! Publishers
    //=======================================================================
    solution_path_rviz_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("solution_path", 10);
    solution_path_pub_ = this->create_publisher<nav_msgs::msg::Path>(solution_path_topic_, 10);
    goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_controller", 10);
    stop_motion_pub_ = this->create_publisher<std_msgs::msg::Bool>("/stop_motion", 10);

    //=======================================================================
    // ! TF
    //=======================================================================
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    //=======================================================================
    // Waiting for odometry
    //=======================================================================

    rclcpp::Rate loop_rate(10);
    while (rclcpp::ok() && !odom_available_)
    {
        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
        RCLCPP_WARN(this->get_logger(), "Waiting for vehicle's odometry");
    }
    RCLCPP_WARN(this->get_logger(), "Odometry received");

    octomap_client_ = this->create_client<GetOctomap>(octomap_service_);

    while (!octomap_client_->wait_for_service(std::chrono::seconds(1)))
    {
        RCLCPP_WARN(this->get_logger(), "Waiting for the Octomap service to become available...");
    }
}

//! Odometry callback
/*!
 * Callback for getting updated vehicle odometry
 */
void PlannFramework::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom_msg)
{
    if (!odom_available_)
        odom_available_ = true;

    geometry_msgs::msg::Pose predictedPose = odom_msg->pose.pose;

    predictedPose.position.x = odom_msg->pose.pose.position.x;

    predictedPose.position.y = odom_msg->pose.pose.position.y;

    tf2::fromMsg(predictedPose, last_robot_pose_);

    double useless_pitch, useless_roll, yaw;
    tf2::Matrix3x3(last_robot_pose_.getRotation()).getEulerYPR(yaw, useless_pitch, useless_roll);

    if ((goal_available_) &&
        sqrt(pow(goal_odom_frame_[0] - last_robot_pose_.getOrigin().getX(), 2.0) +
             pow(goal_odom_frame_[1] - last_robot_pose_.getOrigin().getY(), 2.0)) < (goal_radius_ + 0.3) &&
        abs(yaw - goal_odom_frame_[2]) < (yaw_goal_tolerance_ + 0.08))
    {
        goal_available_ = false;
    }
}

//! Navigation goal callback.
/*!
 * Callback for getting the 2D navigation goal
 */
void PlannFramework::queryGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr nav_goal_msg)
{
    double useless_pitch, useless_roll, yaw;
    tf2::Quaternion q(
        nav_goal_msg->pose.orientation.x,
        nav_goal_msg->pose.orientation.y,
        nav_goal_msg->pose.orientation.z,
        nav_goal_msg->pose.orientation.w);

    tf2::Matrix3x3(q).getRPY(useless_roll, useless_pitch, yaw);

    goal_map_frame_[0] = nav_goal_msg->pose.position.x;
    goal_map_frame_[1] = nav_goal_msg->pose.position.y;
    goal_map_frame_[2] = yaw;

    //=======================================================================
    // Transform from map to odom
    //=======================================================================
    geometry_msgs::msg::TransformStamped tf_map_to_fixed;
    try
    {
        tf_map_to_fixed = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
        return;
    }

    tf2::Transform tf2_map_to_fixed;
    tf2::fromMsg(tf_map_to_fixed.transform, tf2_map_to_fixed);
    tf2::Matrix3x3(tf2_map_to_fixed.getRotation()).getRPY(useless_roll, useless_pitch, yaw);

    tf2::Vector3 goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf2_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.x();
    goal_odom_frame_[1] = goal_point_odom_frame.y();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    solution_path_states_.clear();
    goal_available_ = true;

    goal_pub_->publish(*nav_goal_msg);
}

//!  Planner setup.
/*!
 * Setup a sampling-based planner using OMPL.
 */
void PlannFramework::run()
{

    rclcpp::Rate loop_rate(1 / (timer_period_ - solving_time_)); // 10 hz

    while (rclcpp::ok())
    {
        if (goal_available_)
        {
            planningCallback();
            goal_available_ = false;
        }

        rclcpp::spin_some(this->get_node_base_interface());
        loop_rate.sleep();
    }
}

//!  Periodic callback to solve the query.
/*!
 * Solve the query.
 */
void PlannFramework::planningCallback()
{

    //=======================================================================
    // ! Transform from map to odom
    //=======================================================================
    double useless_pitch, useless_roll, yaw;
    geometry_msgs::msg::TransformStamped tf_map_to_fixed;
    try
    {
        tf_map_to_fixed = tf_buffer_->lookupTransform("map", "odom", tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex)
    {
        RCLCPP_WARN(this->get_logger(), "Could not transform map to odom: %s", ex.what());
        return;
    }

    tf2::Transform tf2_map_to_fixed;
    tf2::fromMsg(tf_map_to_fixed.transform, tf2_map_to_fixed);
    tf2::Matrix3x3(tf2_map_to_fixed.getRotation()).getRPY(useless_roll, useless_pitch, yaw);

    tf2::Vector3 goal_point_odom_frame(goal_map_frame_[0], goal_map_frame_[1], 0.0);
    goal_point_odom_frame = tf2_map_to_fixed.inverse() * goal_point_odom_frame;
    goal_odom_frame_[0] = goal_point_odom_frame.x();
    goal_odom_frame_[1] = goal_point_odom_frame.y();
    goal_odom_frame_[2] = goal_map_frame_[2] - yaw;

    // ! 1. configuration space definition
    //=======================================================================
    // Instantiate the state space
    //=======================================================================
    ob::StateSpacePtr space = ob::StateSpacePtr(new ob::RealVectorStateSpace(2));

    //=======================================================================
    // Set the bounds for the state space
    //=======================================================================
    ob::RealVectorBounds bounds(2);

    if (last_robot_pose_.getOrigin().getX() < goal_odom_frame_[0])
    {
        if (last_robot_pose_.getOrigin().getX() - 10.0 < planning_bounds_x_[0])
            bounds.setLow(0, planning_bounds_x_[0]);
        else
            bounds.setLow(0, last_robot_pose_.getOrigin().getX() - 10.0);

        if (goal_odom_frame_[0] + 5.0 > planning_bounds_x_[1])
            bounds.setHigh(0, planning_bounds_x_[1]);
        else
            bounds.setHigh(0, goal_odom_frame_[0] + 10.0);
    }
    else
    {
        if (last_robot_pose_.getOrigin().getX() + 10.0 > planning_bounds_x_[1])
            bounds.setHigh(0, planning_bounds_x_[1]);
        else
            bounds.setHigh(0, last_robot_pose_.getOrigin().getX() + 10.0);

        if (goal_odom_frame_[0] - 10.0 < planning_bounds_x_[0])
            bounds.setLow(0, planning_bounds_x_[0]);
        else
            bounds.setLow(0, goal_odom_frame_[0] - 10.0);
    }

    if (last_robot_pose_.getOrigin().getY() < goal_odom_frame_[1])
    {
        if (last_robot_pose_.getOrigin().getY() - 10.0 < planning_bounds_y_[0])
            bounds.setLow(1, planning_bounds_y_[0]);
        else
            bounds.setLow(1, last_robot_pose_.getOrigin().getY() - 10.0);

        if (goal_odom_frame_[1] + 10.0 > planning_bounds_y_[1])
            bounds.setHigh(1, planning_bounds_y_[1]);
        else
            bounds.setHigh(1, goal_odom_frame_[1] + 10.0);
    }
    else
    {
        if (last_robot_pose_.getOrigin().getY() + 10.0 > planning_bounds_y_[1])
            bounds.setHigh(1, planning_bounds_y_[1]);
        else
            bounds.setHigh(1, last_robot_pose_.getOrigin().getY() + 10.0);

        if (goal_odom_frame_[1] - 10.0 < planning_bounds_y_[0])
            bounds.setLow(1, planning_bounds_y_[0]);
        else
            bounds.setLow(1, goal_odom_frame_[1] - 10.0);
    }

    space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

    //=======================================================================
    // Define a simple setup class
    //=======================================================================
    simple_setup_ = og::SimpleSetupPtr(new og::SimpleSetup(space));
    ob::SpaceInformationPtr si = simple_setup_->getSpaceInformation();

    // ! 2. Define start and goals
    //=======================================================================
    // Create a start and goal states
    //=======================================================================
    start_state_[0] = double(last_robot_pose_.getOrigin().getX()); // x
    start_state_[1] = double(last_robot_pose_.getOrigin().getY()); // y

    // create a start state
    ob::ScopedState<> start(space);

    start[0] = double(start_state_[0]); // x
    start[1] = double(start_state_[1]); // y

    // create a goal state
    ob::ScopedState<> goal(space);

    goal[0] = double(goal_map_frame_[0]); // x
    goal[1] = double(goal_map_frame_[1]); // y

    // ==================================================================
    // ! requesting octomap service
    // ==================================================================

    RCLCPP_DEBUG(this->get_logger(), "Requesting the map from %s...", octomap_service_.c_str());

    auto request = std::make_shared<GetOctomap::Request>();
    auto result = octomap_client_->async_send_request(request);

    if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        RCLCPP_WARN(this->get_logger(), "Obtained Octomap");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to call service %s", octomap_service_.c_str());
    }

    auto octomap_msg = result.get()->map;

    // ==================================================================

    // ! 3. Set state validity checker
    //=======================================================================
    // Set state validity checking for this space
    //=======================================================================
    ob::StateValidityCheckerPtr om_stat_val_check;
    om_stat_val_check = ob::StateValidityCheckerPtr(
        new OmFclStateValidityCheckerR2(simple_setup_->getSpaceInformation(),
                                        planning_bounds_x_, planning_bounds_y_, robot_base_radius_, octomap_msg));

    // ! 4. set optimization objective
    //=======================================================================
    // Set optimization objective
    //=======================================================================
    simple_setup_->getProblemDefinition()->setOptimizationObjective(ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective(si)));

    // ! 5. Define planner
    //=======================================================================
    // Create a planner for the defined space
    //=======================================================================

    ob::PlannerPtr planner;
    if (planner_name_.compare("RRT") == 0)
        planner = ob::PlannerPtr(new og::RRT(si));
    if (planner_name_.compare("PRMstar") == 0)
        planner = ob::PlannerPtr(new og::PRMstar(si));
    else if (planner_name_.compare("RRTstar") == 0)
        planner = ob::PlannerPtr(new og::RRTstar(si));
    else if (planner_name_.compare("PRM") == 0)
        planner = ob::PlannerPtr(new og::PRM(si));

    // ! 6. Set attributes to simple setup
    //=======================================================================
    // Set the setup planner
    //=======================================================================
    simple_setup_->setPlanner(planner);

    //=======================================================================
    // Set the start and goal states
    //=======================================================================
    simple_setup_->setStartState(start);
    simple_setup_->setGoalState(goal, goal_radius_);
    simple_setup_->setStateValidityChecker(om_stat_val_check);
    simple_setup_->getStateSpace()->setValidSegmentCountFactor(15.0);

    //=======================================================================
    // Perform setup steps for the planner
    //=======================================================================
    simple_setup_->setup();

    // ! 7. attempt to solve
    //=======================================================================
    // Attempt to solve the problem within one second of planning time
    //=======================================================================
    ob::PlannerStatus solved = simple_setup_->solve(solving_time_);

    if (solved && simple_setup_->haveExactSolutionPath())
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path

        og::PathGeometric path = simple_setup_->getSolutionPath();

        // generates varios little segments for the waypoints obtained from the planner
        path.interpolate(int(path.length() / 0.2));

        // path_planning_msgs::PathConstSpeed solution_path;
        RCLCPP_INFO(this->get_logger(), "\n\tpath with cost %f has been found with simple_setup\n",
                    path.cost(simple_setup_->getProblemDefinition()->getOptimizationObjective()).value());

        std::vector<ob::State *> path_states;
        path_states = path.getStates();

        double distance_to_goal =
            sqrt(pow(goal_odom_frame_[0] - path_states[path_states.size() - 1]
                                               ->as<ob::RealVectorStateSpace::StateType>()
                                               ->values[0],
                     2.0) +
                 pow(goal_odom_frame_[1] - path_states[path_states.size() - 1]
                                               ->as<ob::RealVectorStateSpace::StateType>()
                                               ->values[1],
                     2.0));

        if (simple_setup_->haveExactSolutionPath() || distance_to_goal <= goal_radius_)
        {
            // path.interpolate(int(path.length() / 1.0));
            visualizeRRT(path);

            //=======================================================================
            // Controller
            //=======================================================================
            if (path_states.size() > 0)
            {
                nav_msgs::msg::Path solution_path_for_control;
                solution_path_for_control.header.frame_id = world_frame_;
                for (unsigned int i = 0; i < path_states.size(); i++)
                {
                    geometry_msgs::msg::PoseStamped p;
                    p.pose.position.x = path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[0];
                    p.pose.position.y = path_states[i]->as<ob::RealVectorStateSpace::StateType>()->values[1];

                    if (i == (path_states.size() - 1))
                    {
                        if (goal_available_)
                        {
                            tf2::Quaternion myQuaternion;

                            myQuaternion.setRPY(useless_roll, useless_pitch, goal_odom_frame_[2]);

                            myQuaternion = myQuaternion.normalize();

                            p.pose.orientation.x = myQuaternion.getX();
                            p.pose.orientation.y = myQuaternion.getY();
                            p.pose.orientation.z = myQuaternion.getZ();
                            p.pose.orientation.w = myQuaternion.getW();
                        }
                    }
                    solution_path_for_control.poses.push_back(p);
                }
                solution_path_pub_->publish(solution_path_for_control);
            }
            //========================
            // SAVE THE PATH
            // =======================
            solution_path_states_.clear();
            for (int i = 0; i < path_states.size(); i++)
            {
                ob::State *s = space->allocState();
                space->copyState(s, path_states[i]);
                solution_path_states_.push_back(s);
            }
        }
        //=======================================================================
        // Clear previous solution path
        //=======================================================================
        simple_setup_->clear();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "\n\tpath has not been found\n");
    }
}

//! Resulting path visualization.
/*!
 * Visualize resulting path.
 */
void PlannFramework::visualizeRRT(og::PathGeometric &geopath)
{
    // %Tag(MARKER_INIT)%
    tf2::Quaternion orien_quat;
    visualization_msgs::msg::Marker visual_rrt, visual_result_path;
    visual_result_path.header.frame_id = visual_rrt.header.frame_id = world_frame_;
    visual_result_path.header.stamp = visual_rrt.header.stamp = this->now();
    visual_rrt.ns = "planner_rrt";
    visual_result_path.ns = "planner_result_path";
    visual_result_path.action = visual_rrt.action = visualization_msgs::msg::Marker::ADD;

    visual_result_path.pose.orientation.w = visual_rrt.pose.orientation.w = 1.0;
    // %EndTag(MARKER_INIT)%

    // %Tag(ID)%
    visual_rrt.id = 0;
    visual_result_path.id = 1;
    // %EndTag(ID)%

    // %Tag(TYPE)%
    visual_rrt.type = visual_result_path.type = visualization_msgs::msg::Marker::LINE_LIST;
    // %EndTag(TYPE)%

    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    visual_rrt.scale.x = 0.03;
    visual_result_path.scale.x = 0.08;
    // %EndTag(SCALE)%

    // %Tag(COLOR)%
    // Points are green
    visual_result_path.color.g = 1.0;
    visual_result_path.color.a = 1.0;

    // Line strip is blue
    visual_rrt.color.b = 1.0;
    visual_rrt.color.a = 1.0;

    const ob::RealVectorStateSpace::StateType *state_r2;

    geometry_msgs::msg::Point p;

    ob::PlannerData planner_data(simple_setup_->getSpaceInformation());
    simple_setup_->getPlannerData(planner_data);

    std::vector<unsigned int> edgeList;
    RCLCPP_DEBUG(this->get_logger(), "number of states in the tree: %d",
                 planner_data.numVertices());

    if (visualize_tree_)
    {
        for (unsigned int i = 1; i < planner_data.numVertices(); ++i)
        {
            if (planner_data.getVertex(i).getState() && planner_data.getIncomingEdges(i, edgeList) > 0)
            {
                state_r2 = planner_data.getVertex(i).getState()->as<ob::RealVectorStateSpace::StateType>();
                p.x = state_r2->values[0];
                p.y = state_r2->values[1];
                p.z = 0.1;

                visual_rrt.points.push_back(p);

                state_r2 =
                    planner_data.getVertex(edgeList[0]).getState()->as<ob::RealVectorStateSpace::StateType>();
                p.x = state_r2->values[0];
                p.y = state_r2->values[1];
                p.z = 0.1;

                visual_rrt.points.push_back(p);
            }
        }
        solution_path_rviz_pub_->publish(visual_rrt);
    }

    std::vector<ob::State *> states = geopath.getStates();
    for (uint32_t i = 0; i < geopath.getStateCount(); ++i)
    {
        // extract the component of the state and cast it to what we expect

        state_r2 = states[i]->as<ob::RealVectorStateSpace::StateType>();
        p.x = state_r2->values[0];
        p.y = state_r2->values[1];
        p.z = 0.1;

        if (i > 0)
        {
            visual_result_path.points.push_back(p);

            state_r2 = states[i - 1]->as<ob::RealVectorStateSpace::StateType>();
            p.x = state_r2->values[0];
            p.y = state_r2->values[1];
            p.z = 0.1;

            visual_result_path.points.push_back(p);
        }
    }
    solution_path_rviz_pub_->publish(visual_result_path);
}

//! Main function
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto planning_framework = std::make_shared<PlannFramework>();

    planning_framework->run();
    return 0;
}
