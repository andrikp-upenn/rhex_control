#include "rover.h"

Rover::Rover(std::vector<int> motor_can_ids, float wheel_radius, float wheel_base) 
    : RobotBase(motor_can_ids),
      wheel_radius_(wheel_radius),
      wheel_base_(wheel_base),
      diff_drive_{wheel_radius, wheel_base} {}


void Rover::velocityControl(float linear, float angular, float dt) {
    auto wheel_velocities = diff_drive_.setVelocity(linear, angular);
    // Start timer (tic)
    float torqueCalculation;
    for (int i = 0; i < motors_.size(); i++) {
        if(i % 2 == 0) {
            torqueCalculation = pid_controllers_[i].calculateWithFeedForward(
                motors_[i].getMotorData().speed, 
                0, // derivative set to zero for PI velocity control
                -wheel_velocities.first,
                0, // derivative set to zero for PI velocity control
                0, 
                dt);
            motors_[i].sendCommandMITMode(0, 0, 0, 0, torqueCalculation);

        } else {
            torqueCalculation = pid_controllers_[i].calculateWithFeedForward(
                motors_[i].getMotorData().speed, 
                0, // derivative set to zero for PI velocity control
                wheel_velocities.second, 
                0, // derivative set to zero for PI velocity control
                0, 
                dt);
            motors_[i].sendCommandMITMode(0, 0, 0, 0, torqueCalculation);
        }
    }


}

void Rover::positionControl(float angle) {
    //Position control
    for (int i = 0; i < motors_.size(); i++) {
        motors_[i].sendCommandMITMode(angle, 0, kp_, kd_, 0);
    }

}