#include "robot_base.h"

RobotBase::RobotBase(std::vector<int> motor_can_ids){
    openSocket();

    motor_can_ids_ = motor_can_ids;
    for (int i = 0; i < motor_can_ids.size(); i++) {
        motors_.push_back(CubemarsControl(motor_can_ids[i], sock_));
        RCLCPP_INFO(logger_, "Entering MIT mode...");
        motors_[i].enterMITMode();
        RCLCPP_INFO(logger_, "Entered MIT mode");
    }

    for(int i = 0; i < motor_can_ids.size(); i++) {
        pid_controllers_.push_back(PIDController(kp_, ki_, kd_));
    }

}

RobotBase::~RobotBase() {
    // Do nothing
}

void RobotBase::setMotorsIdle() {
    for (auto& motor : motors_) {
        motor.sendCommandMITMode(0, 0, 0, 0, 0);
    }
}

void RobotBase::zeroMotors() {
    for (auto& motor : motors_) {
        RCLCPP_INFO(logger_, "Zeroing motor...");
        motor.zeroMotor();
        RCLCPP_INFO(logger_, "Zeroed motor");
    }
}

void RobotBase::stopMotors() {
    for (auto& motor : motors_) {
        motor.sendCommandMITMode(0, 0, 0, 0, 0);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Small delay
    }
    for(auto& motor : motors_){
        RCLCPP_INFO(logger_, "Exiting MIT mode...");
        motor.exitMITMode();
        RCLCPP_INFO(logger_, "Exited MIT mode");
    }
}

void RobotBase::setKp(float kp) {
    kp_ = kp;
    for(int i = 0; i < motor_can_ids_.size(); i++) {
        pid_controllers_[i].setGains(kp_, ki_, kd_);
    }
}

void RobotBase::setKd(float kd) {
    kd_ = kd;
    for(int i = 0; i < motor_can_ids_.size(); i++) {
        pid_controllers_[i].setGains(kp_, ki_, kd_);
    }
}

void RobotBase::setKi(float ki) {
    ki_ = ki;
    for(int i = 0; i < motor_can_ids_.size(); i++) {
        pid_controllers_[i].setGains(kp_, ki_, kd_);
    }
}

std::vector<CubemarsControl::Cubemars_Motor> RobotBase::readMotorValues() {
    std::vector<CubemarsControl::Cubemars_Motor> motor_values;
    for (auto& motor : motors_) {
        motor_values.push_back(motor.getMotorData());
    }
    return motor_values;
}

CubemarsControl::Cubemars_Motor RobotBase::readMotorValue(int motor_id) {
    for (auto& motor : motors_) {
        if (motor.getMotorID() == motor_id) {
            return motor.getMotorData();
        }
    }
    CubemarsControl::Cubemars_Motor motor;
    return motor;
}

std::vector<CubemarsControl>& RobotBase::getMotors() {
    return motors_;
}

int RobotBase::openSocket() {
    // 1.Create socket
    RCLCPP_INFO(logger_, "Opening Socket...");
    sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0)
    {
        perror("socket PF_CAN failed");
        exit(0);
        return 1;
    }

    // 2.Specify can0 device
    strcpy(ifr_.ifr_name, "can0");
    ret_ = ioctl(sock_, SIOCGIFINDEX, &ifr_);
    if (ret_ < 0)
    {
        perror("ioctl failed");
        exit(0);
        return 1;
    }

    // 3.Bind the socket to can0
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    ret_ = bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_));
    if (ret_ < 0)
    {
        perror("bind failed");
        exit(0);
        return 1;
    }
    RCLCPP_INFO(logger_, "Opened Socket");
    return 0;
}

int RobotBase::closeSocket() {
    RCLCPP_INFO(logger_, "Closing Socket...");
    close(sock_);
    RCLCPP_INFO(logger_, "Closed Socket");
    return 0;
}