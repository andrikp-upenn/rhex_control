#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <algorithm>
#include <iostream>
#include <array>
#include <cmath>

class PIDController{
    public:
        PIDController(float kp, float ki, float kd);

        float calculateWithFeedForward(float current_position, float current_velocity, float desired_position, float desired_velocity, float feed_forward, float dt);
        /*
        float calculate(float current_position, float current_velocity, float desired_position, float desired_velocity, float dt);
        float calculateWithGains(float current_position, float current_velocity, float desired_position, float desired_velocity, float kp, float ki, float kd, float dt);
        float calculateWithFeedForwardAndGains(float current_position, float current_velocity, float desired_position, float desired_velocity, float feed_forward, float kp, float ki, float kd, float dt);
        */
        void setGains(float kp, float ki, float kd);
        void setMaxIntegral(float max_integral_term){max_integral_term_ = max_integral_term;};
        void setMaxOutput(float max_output){max_output_ = max_output;};
        void resetIntegral(){integral_ = 0;};
        void setIntegral(float integral){integral_ = integral;};
        void setKp(float kp){kp_ = kp;};
        void setKi(float ki){ki_ = ki;};
        void setKd(float kd){kd_ = kd;};
        std::array<float, 3> getGains(){return {kp_, ki_, kd_};};
        float getIntegral(){return integral_;};
        float getMaxIntegral(){return max_integral_term_;};
        float getMaxOutput(){return max_output_;};
        float getKp(){return kp_;};
        float getKi(){return ki_;};
        float getKd(){return kd_;};

        
    private:
        float kp_, ki_, kd_;
        float integral_;
        float max_integral_term_ = 2;
        float max_output_ = 30;
};

#endif