#ifndef ROVER_H
#define ROVER_H

#include <chrono>
#include <iostream>
#include "diff_drive.h"
#include "robot_base.h"


class Rover : public RobotBase{

public:
    Rover(std::vector<int> motor_can_ids, float wheel_radius, float wheel_base);

    void velocityControl(float linear, float angular, float dt);
    void positionControl(float angle);

private:

    DiffDrive diff_drive_;

    float wheel_radius_;
    float wheel_base_;

};

#endif