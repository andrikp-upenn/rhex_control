#include "diff_drive.h"

DiffDrive::DiffDrive(float wheel_radius, float wheel_base)
    : wheel_radius_(wheel_radius), wheel_base_(wheel_base), 
      left_wheel_velocity_(0), right_wheel_velocity_(0) {}

std::pair<float,float> DiffDrive::setVelocity(float linear_vel, float angular_vel) {
    // Calculate the left and right wheel velocities
    left_wheel_velocity_ = (linear_vel - (angular_vel * wheel_base_ / 2.0)) / wheel_radius_;
    right_wheel_velocity_ = (linear_vel + (angular_vel * wheel_base_ / 2.0)) / wheel_radius_;
    return std::make_pair(left_wheel_velocity_, right_wheel_velocity_);
}

float DiffDrive::getLeftWheelVelocity(){
    return left_wheel_velocity_;
}

float DiffDrive::getRightWheelVelocity(){
    return right_wheel_velocity_;
}