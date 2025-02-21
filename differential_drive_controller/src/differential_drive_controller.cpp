#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

class DifferentialDriveController : public rclcpp::Node
{
public:
    DifferentialDriveController() : Node("differential_drive_controller")
    {
        this->declare_parameter<double>("wheelbase", 0.5); // meters
        this->declare_parameter<double>("wheel_radius", 0.1); // meters
        this->declare_parameter<double>("max_rpm", 300.0); // rpm

        cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&DifferentialDriveController::cmd_vel_callback, this, std::placeholders::_1));

        left_wheel_rpm_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/left_wheel_rpm", 10);
        right_wheel_rpm_publisher_ = this->create_publisher<std_msgs::msg::Float64>("/right_wheel_rpm", 10);
    }

private:
    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        double wheelbase = this->get_parameter("wheelbase").as_double();
        double wheel_radius = this->get_parameter("wheel_radius").as_double();
        double max_rpm = this->get_parameter("max_rpm").as_double();

        double linear_velocity = msg->linear.x;
        double angular_velocity = msg->angular.z;

        double left_velocity = (linear_velocity - (wheelbase / 2.0) * angular_velocity) / wheel_radius;
        double right_velocity = (linear_velocity + (wheelbase / 2.0) * angular_velocity) / wheel_radius;

        double left_rpm = left_velocity * 60.0 / (2 * M_PI);
        double right_rpm = right_velocity * 60.0 / (2 * M_PI);

        left_rpm = std::min(left_rpm, max_rpm);
        right_rpm = std::min(right_rpm, max_rpm);

        std_msgs::msg::Float64 left_rpm_msg;
        std_msgs::msg::Float64 right_rpm_msg;

        left_rpm_msg.data = left_rpm;
        right_rpm_msg.data = right_rpm;

        left_wheel_rpm_publisher_->publish(left_rpm_msg);
        right_wheel_rpm_publisher_->publish(right_rpm_msg);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr left_wheel_rpm_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr right_wheel_rpm_publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DifferentialDriveController>());
    rclcpp::shutdown();
    return 0;
}
