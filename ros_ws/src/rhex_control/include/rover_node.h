#ifndef ROVER_NODE_H
#define ROVER_NODE_H

#include <rclcpp/rclcpp.hpp>

#include "trusses_custom_interfaces/srv/set_value.hpp"
#include "trusses_custom_interfaces/srv/set_string.hpp"
#include "trusses_custom_interfaces/srv/set_value_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "robot_base.h"
#include "rover.h"

#include <cstdlib>
#include <csignal>
#include <chrono>
#include <string>
#include <vector>
#include <iostream>
#include <map>


using namespace std::chrono_literals;

class RoverNode : public rclcpp::Node
{
public:
    explicit RoverNode(int rover_number);
    void cleanup();

private:
    bool zero_motor_flag_ = false;
    float des_linear_vel_ = 0;
    float des_angular_vel_ = 0;

    std::string behavior_ = "idle";
    
    enum BehaviorTypes{
        kIdle,
        kVelocity,
        kPosition
    };

    struct PIDGains{
        float kp;
        float ki;
        float kd;
    };

    std::map<std::string, BehaviorTypes> behavior_map_ = {
        {"idle", kIdle},
        {"velocity", kVelocity},
        {"position", kPosition}
    };

    std::map<std::string, PIDGains> behavior_gains_ = {
        {"idle", {0,0,0}},
        {"velocity", {8,.03,0}},
        {"position", {1,0,.1}}
    };

    int control_loop_counter;

    float wheel_radius_ = .3;
    float wheel_base_ = .58;
    std::vector<int> motor_can_ids_ = {0xD,0xC,0xF,0xA};

    Rover rover_;

    rclcpp::TimerBase::SharedPtr control_loop_timer_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_all_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetValueArray>::SharedPtr set_behavior_gains_service_;
    rclcpp::Service<trusses_custom_interfaces::srv::SetString>::SharedPtr set_behavior_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_node_service_;

    void control_loop();

    void stop_all_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void stop_node_callback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    void set_behavior_gains_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Response> response);

    void set_behavior_callback(
        const std::shared_ptr<trusses_custom_interfaces::srv::SetString::Request> request,
        std::shared_ptr<trusses_custom_interfaces::srv::SetString::Response> response);

    // Publishers
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr curr_state_pub_;
    void publish_curr_state();

    // Subscribers
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr set_states_sub_;
    void set_states_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
};
#endif