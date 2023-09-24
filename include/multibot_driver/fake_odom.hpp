#pragma once

#include <array>
#include <string>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/msg/tf_message.hpp>

namespace ISR_M2
{
    enum WHEEL{LEFT, RIGHT};

    class FakeOdom: public rclcpp::Node
    {
    private:
        void command_velocity_callback(
            const geometry_msgs::msg::Twist::SharedPtr _cmd_vel_msg);
        void update_callback();
    
    private:
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
        rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

        rclcpp::TimerBase::SharedPtr update_timer_;

    private:
        void init_parameters();
        void init_varibales();

        void calculate_odometry(const rclcpp::Duration &_duration);
        void publish(const rclcpp::Time &_now);

    private:
        double wheel_seperation_;
        double wheel_radius_;

        std::string namespace_;

        std::string frame_id_of_odometry_;
        std::string child_frame_id_of_odometry_;

        std::array<double, 2> wheel_speed_;
        std::array<double, 2> last_velocity_;
        std::array<double, 2> last_position_;

        std::array<double, 3> robot_pose_;
        std::array<double, 3> robot_vel_;

        double linear_velocity_;
        double angular_velocity_;

        rclcpp::Time prev_cmd_vel_time_;
        double cmd_vel_timeout_;

        bool use_gazebo_odom_;
    
    public:
        FakeOdom();
        ~FakeOdom();
    }; // class FakeOdom
} // class ISR_M2