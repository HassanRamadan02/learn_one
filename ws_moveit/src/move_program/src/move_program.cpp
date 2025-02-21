// Include necessary headers
#include <rclcpp/rclcpp.hpp>                      // ROS 2 C++ client library
#include <moveit/move_group_interface/move_group_interface.h>  // MoveIt motion planning interface
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // For handling geometric transformations
#include <tf2/LinearMath/Quaternion.h>            // Quaternion math library
#include <iostream>                               // For terminal input/output
#include <sstream>                                // For string stream processing
#include <thread>                                 // For multi-threading support
#include <memory>                                 // For smart pointers

int main(int argc, char* argv[]) {
    // Initialize ROS 2 communication
    rclcpp::init(argc, argv);
    
    // Create ROS node with parameter declaration capabilities
    auto const node = std::make_shared<rclcpp::Node>(
        "move_param",
        rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true)
    );
    
    // Get logger for node status messages
    auto const logger = rclcpp::get_logger("move_param");

    // Set up ROS executor in a separate thread to handle callbacks while waiting for user input
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor->add_node(node);
    std::thread executor_thread([executor]() { executor->spin(); });

    // Initialize MoveIt interface for the Panda arm planning group
    moveit::planning_interface::MoveGroupInterface move_group(node, "panda_arm");

    // Main control loop
    while (rclcpp::ok()) {
        // Prompt user for input
        std::cout << "Enter position and orientation (x y z roll pitch yaw) in radians, or 'exit':\n";
        std::string input_line;
        std::getline(std::cin, input_line);

        // Exit condition check
        if (input_line == "exit") break;

        // Parse input values
        double x, y, z, roll, pitch, yaw;
        std::istringstream iss(input_line);
        
        // Validate and process input
        if (!(iss >> x >> y >> z >> roll >> pitch >> yaw)) {
            std::cout << "Invalid input. Please enter 6 numbers separated by spaces.\n";
            continue;
        }

        // Convert Euler angles (roll, pitch, yaw) to quaternion for orientation
        tf2::Quaternion tf2_quat;
        tf2_quat.setRPY(roll, pitch, yaw);  // Create quaternion from RPY angles
        geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(tf2_quat);  // Convert to ROS message type

        // Define target end-effector pose
        geometry_msgs::msg::Pose target_pose;
        target_pose.orientation = msg_quat;   // Set orientation (quaternion)
        target_pose.position.x = x;           // X coordinate (meters)
        target_pose.position.y = y;           // Y coordinate (meters)
        target_pose.position.z = z;           // Z coordinate (meters)

        // Set the target pose for motion planning
        move_group.setPoseTarget(target_pose);

        // Create motion plan object
        moveit::planning_interface::MoveGroupInterface::Plan plan;

        // Attempt to create a motion plan
        if (move_group.plan(plan)) {
            // Execute the planned trajectory if successful
            move_group.execute(plan);
            RCLCPP_INFO(logger, "Motion executed successfully!");
        } else {
            // Handle planning failure
            RCLCPP_ERROR(logger, "Planning failed! Check target pose and current state.");
        }
    }

    // Clean shutdown sequence
    executor->cancel();         // Stop executor
    executor_thread.join();     // Wait for executor thread to finish
    rclcpp::shutdown();         // Properly shut down ROS
    return 0;
}
