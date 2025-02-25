#ifndef ALTERNATING_TRIPOD_H
#define ALTERNATING_TRIPOD_H

#include <vector>
#include "buehler_clock.h"
#include <unordered_map>
#include <array>
#include <iostream>
#include <cmath>
#include <algorithm>

class AlternatingTripod {
public:
    
    enum LeftRight {
        kLeft,
        kRight
    };

    enum Tripod {
        kTripodA,
        kTripodB
    };
    
    const std::unordered_map<Tripod, std::array<int,3>> kTripodIndices = {
        {kTripodA, {0, 3, 4}},
        {kTripodB, {1, 2, 5}}
    };
    const std::unordered_map<LeftRight, std::array<int,3>> kLeftRightIndices = {
        {kLeft, {0, 2, 4}},
        {kRight, {1, 3, 5}}
    };

    AlternatingTripod(float leg_length, float leg_separation);
    ~AlternatingTripod();

    void initialize();
    void update();
    std::array<float, 6> getLegAngles();
    float getStanceTime(float des_velocity, float phi_s, float leg_length);
    std::pair<float, float> getTurningParamOffsets(float des_angular);
    std::tuple<float,float,float> updateBuehlerFromVelocityCommand(float des_linear, float angular);
    void stand();


private:
    void moveLegs(const std::vector<int>& leg_indices);
    std::array<BuehlerClock, 6> buehler_clocks;
    float master_time_ = 0;

    const std::unordered_map<size_t, LeftRight> index_to_tripod = {
        {0, kLeft},
        {1, kRight},
        {2, kLeft},
        {3, kRight},
        {4, kLeft},
        {5, kRight}
    };
    const std::unordered_map<size_t, Tripod> index_to_side = {
    };
    float angleWrap(float angle, int rotations_before_wrap, bool include_negatives);
    float angularError(float angle1, float angle2, float direction_override);
};
// GET CAN IDS

void AlternatingTripod::initialize() {
    // Initialization code for the robot's legs
}

std::array<float, 6> AlternatingTripod::getLegAngles(){
    std::array<float, 6> leg_angles;
    for (int i = 0; i < 6; i++) {
        leg_angles[i] = buehler_clocks[i % 2].getLegAngle(master_time_);
    }
    return leg_angles;
}

void AlternatingTripod::moveLegs(const std::vector<int>& leg_indices) {
    // Code to move the legs specified by leg_indices
}

void AlternatingTripod::stand() {
    // Code to make the robot stand
}

std::tuple<float,float,float> AlternatingTripod::updateBuehlerFromVelocityCommand(float des_linear, float angular, const BuehlerClock& buehler_clock={}){
    // Code to set the robot's velocity
    float t_s_des = getStanceTime(des_linear);
    auto [delta_t_s,  delta_phi_o] = getTurningParamOffsets(angular);
    return std::make_tuple(t_s_des, delta_t_s, delta_phi_o);
}

float AlternatingTripod::getStanceTime(float des_velocity, float phi_s, float leg_length){
    float t_s_des = phi_s * des_velocity / leg_length;
    return t_s_des;
}

std::pair<float, float> AlternatingTripod::getTurningParamOffsets(float des_angular){
    // Code to calculate the turning parameter offsets
    //TODO: Implement this function
    return {0, 0};
}

float AlternatingTripod::angleWrap(float angle, int rotations_before_wrap, bool include_negatives) {
        float offset = include_negatives * rotations_before_wrap;
        float wrapped_angle = fmod(angle + include_negatives * wrapped_angle, rotations_before_wrap * 2 * M_PI)- include_negatives * wrapped_angle;
        
        return wrapped_angle;
}

float AlternatingTripod::angularError(float angle1, float angle2, float direction_override) {
    float kRotationsBeforeWrap = 1;
    float error = angleWrap(angle1 - angle2, kRotationsBeforeWrap, true);
    return error;
}

#endif