#ifndef ALTERNATING_TRIPOD_H
#define ALTERNATING_TRIPOD_H

#include <vector>
#include <unordered_map>
#include <array>
#include <iostream>
#include <cmath>
#include <algorithm>
#include "buehler_clock.h"

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
    
    // Leg indices for each tripod group
    static const std::unordered_map<Tripod, std::array<int,3>> kTripodIndices;
    
    // Leg indices for left and right sides
    static const std::unordered_map<LeftRight, std::array<int,3>> kLeftRightIndices;

    // Constructor with default leg parameters
    AlternatingTripod(float leg_length = 0.35, float leg_separation = 0.3); // New leg .35cm, leg seperation?
    ~AlternatingTripod();

    // Initialize the gait parameters
    void initialize();
    
    // Update the gait state based on time
    void update(float dt);
    
    // Get the current leg angles for all six legs
    std::array<float, 6> getLegAngles();
    
    // Calculate the stance time based on desired velocity
    float getStanceTime(float des_velocity, float phi_s = M_PI/2, float leg_length = 0.35); // New leg .35cm
    
    // Calculate turning parameter offsets based on desired angular velocity
    std::pair<float, float> getTurningParamOffsets(float des_angular);
    
    // Update Buehler clock parameters from velocity command
    std::tuple<float,float,float> updateBuehlerFromVelocityCommand(float des_linear, float des_angular);
    
    // Command the robot to stand
    void stand();

private:
    // Move the specified legs
    void moveLegs(const std::vector<int>& leg_indices);
    
    // Array of Buehler clocks for each leg
    std::array<BuehlerClock, 6> buehler_clocks_;
    
    // Current time within the gait cycle
    float master_time_ = 0;
    
    // Leg physical parameters
    float leg_length_;
    float leg_separation_;
    
    // Map from leg index to left/right side
    static const std::unordered_map<size_t, LeftRight> index_to_side_;
    
    // Map from leg index to tripod group
    static const std::unordered_map<size_t, Tripod> index_to_tripod_;
    
    // Utility functions for angle math
    float angleWrap(float angle, int rotations_before_wrap = 1, bool include_negatives = true);
    float angularError(float angle1, float angle2, float direction_override = 0);
};

#endif