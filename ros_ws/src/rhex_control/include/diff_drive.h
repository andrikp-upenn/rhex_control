#ifndef DIFFDRIVE_H
#define DIFFDRIVE_H

#include <utility>


class DiffDrive {
public:
    DiffDrive(float wheel_radius, float wheel_base);

    [[maybe_unused]] std::pair<float,float> setVelocity(float linear, float angular);

    float getLeftWheelVelocity();
    float getRightWheelVelocity();
private:
    float wheel_radius_;
    float wheel_base_;
    float left_wheel_velocity_;
    float right_wheel_velocity_;

    void updateWheelVelocities();
};
#endif