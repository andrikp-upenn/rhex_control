#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <rclcpp/rclcpp.hpp>

#include <stdlib.h>
#include <vector>
#include <cstdlib>
#include <csignal>
#include <chrono>
#include <thread> 
#include "cubemars_control.h"
#include "pid_controller.h"


class RobotBase {
public:
    RobotBase(std::vector<int> motor_can_ids);
    virtual ~RobotBase();

    virtual void velocityControl(float linear, float angular, float dt) = 0;
    virtual void positionControl(float angle) = 0;
    virtual void setMotorsIdle();
    virtual void stopMotors();
    virtual void zeroMotors();
    virtual int openSocket();
    virtual int closeSocket();
    virtual void setKd(float kd);
    virtual void setKp(float kp);
    virtual void setKi(float ki);
    virtual std::vector<CubemarsControl::Cubemars_Motor> readMotorValues();
    virtual CubemarsControl::Cubemars_Motor readMotorValue(int motor_id);
    virtual std::vector<CubemarsControl>& getMotors();

protected:

    rclcpp::Logger logger_ = rclcpp::get_logger("RobotBase");
    std::vector<PIDController> pid_controllers_;
    std::vector<int> motor_can_ids_;
    std::vector<CubemarsControl> motors_;
    float kp_ = 0;
    float kd_ = 0;
    float ki_ = 0;
    int sock_;
    int nbytes_;
    int ret_;
    struct sockaddr_can addr_;
    struct ifreq ifr_;

};

#endif