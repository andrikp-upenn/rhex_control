#include "pid_controller.h"

PIDController::PIDController(float kp, float ki, float kd) : kp_(kp), ki_(ki), kd_(kd), integral_(0) {}

float PIDController::calculateWithFeedForward(float current_state, float current_state_derivative, float desired_current_state, float desired_current_state_derivative, float feed_forward, float dt)
{
    float error = desired_current_state - current_state;
    float derivative = desired_current_state_derivative - current_state_derivative;

    if (ki_ != 0){
        integral_ += error * dt;
        float integral_term = ki_ * integral_;
        integral_ = std::clamp<float>(integral_, -max_integral_term_ / ki_, max_integral_term_ / ki_);

    }else{
        integral_ = 0;
    }

    float output_ = kp_ * error + ki_ * integral_ + kd_ * derivative + feed_forward;
    
    output_ = std::clamp<float>(output_, -max_output_, max_output_);
    return output_;
};
/*
float PIDController::calculate(float current_state, float current_state_derivative, float desired_current_state, float desired_current_state_derivative, float dt){
    float error = desired_current_state - current_state;
    integral_ += error * dt;
    float derivative = desired_current_state_derivative - current_state_derivative;
    float integral_term = ki_ * integral_;

    if(ki_ == 0){
    }else if (integral_term > max_integral_term_) {
        integral_ = max_integral_term_ / ki_;
    } else if (integral_term < -max_integral_term_) {
        integral_ = -max_integral_term_ / ki_;
    }

    float output_ = kp_ * error + ki_ * integral_ + kd_ * derivative;
    prevError_ = error;
    if(output_ > max_output_){
        output_ = max_output_;
    }else if(output_ < -max_output_){
        output_ = -max_output_;
    }
    return output_;
};

float PIDController::calculateWithGains(float current_state, float current_state_derivative, float desired_current_state, float desired_current_state_derivative, float kp, float ki, float kd, float dt){
    float error = desired_current_state - current_state;
    integral_ += error * dt;
    float derivative = desired_current_state_derivative - current_state_derivative;
    float integral_term = ki * integral_;

    if(ki == 0){
    }else if (integral_term > max_integral_term_) {
        integral_ = max_integral_term_ / ki;
    } else if (integral_term < -max_integral_term_) {
        integral_ = -max_integral_term_ / ki;
    }

    float output_ = kp * error + ki * integral_ + kd * derivative;
    prevError_ = error;
    if(output_ > max_output_){
        output_ = max_output_;
    }else if(output_ < -max_output_){
        output_ = -max_output_;
    }
    return output_;
};

float PIDController::calculateWithFeedForwardAndGains(float current_state, float current_state_derivative, float desired_current_state, float desired_current_state_derivative, float feed_forward, float kp, float ki, float kd, float dt){
    float error = desired_current_state - current_state;
    integral_ += error * dt;
    float derivative = desired_current_state_derivative - current_state_derivative;
    float integral_term = ki * integral_;

    if(ki == 0){
    }else if (integral_term > max_integral_term_) {
        integral_ = max_integral_term_ / ki;
    } else if (integral_term < -max_integral_term_) {
        integral_ = -max_integral_term_ / ki;
    }

    float output_ = kp * error + ki * integral_ + kd * derivative + feed_forward;
    prevError_ = error;
    if(output_ > max_output_){
        output_ = max_output_;
    }else if(output_ < -max_output_){
        output_ = -max_output_;
    }
    return output_;
};
*/

void PIDController::setGains(float kp, float ki, float kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}