#include "rover_node.h"

void signalHandler(int signum) {
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Interrupt signal received, shutting down...");
    rclcpp::shutdown();
}

RoverNode::RoverNode(int rover_number) : Node("rover" + std::to_string(rover_number)), rover_{motor_can_ids_, wheel_radius_, wheel_base_}{

    // Initialize services
    stop_all_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_all", std::bind(&RoverNode::stop_all_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_behavior_gains_service_ = this->create_service<trusses_custom_interfaces::srv::SetValueArray>(
        "~/set_behavior_gains", std::bind(&RoverNode::set_behavior_gains_callback, this, std::placeholders::_1, std::placeholders::_2));
    set_behavior_service_ = this->create_service<trusses_custom_interfaces::srv::SetString>(
        "~/set_behavior", std::bind(&RoverNode::set_behavior_callback, this, std::placeholders::_1, std::placeholders::_2));
    stop_node_service_ = this->create_service<std_srvs::srv::Trigger>(
        "~/stop_node", std::bind(&RoverNode::stop_node_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Initialize publishers
    curr_state_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("~/current_state", 10);

    //Initialize control loop
    
    control_loop_timer_ = this->create_wall_timer(
        5ms, //Period
        std::bind(&RoverNode::control_loop, this)
    );
    
    
    // Initialize Subscriber
    set_states_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "~/set_states", 10,
        std::bind(&RoverNode::set_states_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "Rover Node initialized with rover number: %d", rover_number);
}

void RoverNode::cleanup() {
    RCLCPP_INFO(this->get_logger(), "Cleaning up Rover Node.");
    
    // Stop the control loop timer
    control_loop_timer_->cancel();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Small delay

    // Additional cleanup code here if needed
    rover_.stopMotors();

    std::this_thread::sleep_for(std::chrono::milliseconds(500));  // Small delay

    rover_.closeSocket();
}

void RoverNode::stop_node_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void) request; // Silence unused parameter warning
    response->success = true;
    response->message = "Node attempting to stop.";
    RCLCPP_INFO(this->get_logger(), "stop_node service called.");
    rclcpp::shutdown();
}


void RoverNode::stop_all_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void) request; // Silence unused parameter warning
    rover_.stopMotors();
    response->success = true;
    response->message = "All systems stopped.";
    RCLCPP_INFO(this->get_logger(), "stop_all service called.");
}

void RoverNode::set_behavior_gains_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetValueArray::Response> response)
{
    behavior_gains_[behavior_].kp = request->data[0];
    behavior_gains_[behavior_].ki = request->data[1];
    behavior_gains_[behavior_].kd = request->data[2];

    rover_.setKp(request->data[0]);
    rover_.setKi(request->data[1]);
    rover_.setKd(request->data[2]);
    RCLCPP_INFO(this->get_logger(), "Set_behavior gains service called.");
}

void RoverNode::set_behavior_callback(
    const std::shared_ptr<trusses_custom_interfaces::srv::SetString::Request> request,
    std::shared_ptr<trusses_custom_interfaces::srv::SetString::Response> response)
{
    RCLCPP_INFO(this->get_logger(), "Set_behavior service called with data: %s", request->data.c_str());
    //TODO add error checking for invalid behavior change
    
    if(behavior_map_.find(request->data) == behavior_map_.end()){
        RCLCPP_ERROR(this->get_logger(), "Invalid behavior change request: %s", request->data.c_str());
        //response->success = false;
        //response->message = "Invalid behavior change request.";
        return;
    }

    behavior_ = request->data;

    rover_.setKp(behavior_gains_[behavior_].kp);
    rover_.setKi(behavior_gains_[behavior_].ki);
    rover_.setKd(behavior_gains_[behavior_].kd);

    switch(behavior_map_[behavior_]){
        case kPosition:
            zero_motor_flag_ = true;
            break;
        default:
            break;

    }
        

    RCLCPP_INFO(this->get_logger(), "Set_behavior service called.");
}

void RoverNode::publish_curr_state() {
    std_msgs::msg::Float32MultiArray msg;
    msg.data.clear();
    // std::vector<CubemarsControl::Cubemars_Motor> MotorVals = rover_.readMotorValues();
    auto& motors = rover_.getMotors();
    // for (int i = 0; i < MotorVals.size(); i++) {
    //     msg.data.push_back(MotorVals[i].position);
    //     msg.data.push_back(MotorVals[i].speed);
    //     msg.data.push_back(MotorVals[i].torque);
    //     msg.data.push_back(MotorVals[i].temperature);
        
    // }
    for (int i = 0; i < motors.size(); i++) {
        msg.data.emplace_back(motors[i].getMotorData().position);
        msg.data.emplace_back(motors[i].getMotorData().speed);
        msg.data.emplace_back(motors[i].getMotorData().torque);
        msg.data.emplace_back(motors[i].getMotorData().temperature);
    }
    curr_state_pub_->publish(msg);
}

void RoverNode::control_loop() {
    control_loop_counter++;
    switch(behavior_map_[behavior_])
    {
    default:
    case kIdle:
        rover_.setMotorsIdle();
        break;
    case kVelocity:
        rover_.velocityControl(des_linear_vel_, des_angular_vel_, 0.005);
        break;
    case kPosition:
        if(zero_motor_flag_){
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            rover_.zeroMotors();
            zero_motor_flag_ = false ;
        }
        rover_.positionControl(0);
        break;
    }

    if(control_loop_counter >= 4){
        publish_curr_state();
        control_loop_counter = 0;
    }
    
}

void RoverNode::set_states_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {

    des_linear_vel_ = msg->linear.x;
    des_angular_vel_ = msg->angular.z;
    //RCLCPP_INFO(this->get_logger(), "Set states callback: linear =%f angular =%f", 
    //            msg->linear.x, msg->angular.z);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    int rover_number = -1;

    // Loop through argv to find the custom argument
    for (int i = 1; i < argc; ++i) {  
        std::string arg(argv[i]);
        if (arg == "--rover_number" && i + 1 < argc) {
            rover_number = std::atoi(argv[i + 1]);
            i++; 
        }
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Rover number: %d", rover_number);

    // Register signal handler for SIGINT
    std::signal(SIGINT, signalHandler);

    // Create and spin the node
    auto executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

    auto rover_node = std::make_shared<RoverNode>(rover_number);
    executor->add_node(rover_node);

    rclcpp::on_shutdown([rover_node]() {
        rover_node->cleanup();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Small delay
    });
    
    executor->spin();
    return 0;
}
