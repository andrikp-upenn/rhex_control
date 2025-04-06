#include "alternating_tripod.h"

// Static member initialization
const std::unordered_map<AlternatingTripod::Tripod, std::array<int,3>> AlternatingTripod::kTripodIndices = {
    {kTripodA, {0, 3, 4}}, // Front-left, middle-right, back-left
    {kTripodB, {1, 2, 5}}  // Front-right, middle-left, back-right
};

const std::unordered_map<AlternatingTripod::LeftRight, std::array<int,3>> AlternatingTripod::kLeftRightIndices = {
    {kLeft, {0, 2, 4}},   // Left side legs (front, middle, back)
    {kRight, {1, 3, 5}}   // Right side legs (front, middle, back)
};

// Completed static member maps
const std::unordered_map<size_t, AlternatingTripod::LeftRight> AlternatingTripod::index_to_side_ = {
    {0, kLeft},  // Front-left
    {1, kRight}, // Front-right
    {2, kLeft},  // Middle-left
    {3, kRight}, // Middle-right
    {4, kLeft},  // Back-left
    {5, kRight}  // Back-right
};

const std::unordered_map<size_t, AlternatingTripod::Tripod> AlternatingTripod::index_to_tripod_ = {
    {0, kTripodA}, // Front-left
    {1, kTripodB}, // Front-right
    {2, kTripodB}, // Middle-left
    {3, kTripodA}, // Middle-right
    {4, kTripodA}, // Back-left
    {5, kTripodB}  // Back-right
};

// Construct a new Alternating Tripod object
AlternatingTripod::AlternatingTripod(float leg_length, float leg_separation)
    : leg_length_(leg_length), leg_separation_(leg_separation)
{
    // Initialize clocks with defaults
    initialize();
}

AlternatingTripod::~AlternatingTripod()
{
    // No dynamic resources to clean up
}

// Initialize the Buehler clocks for each leg
void AlternatingTripod::initialize()
{
    // Default Buehler clock parameters
    float phi_s = M_PI/2;   // Stance phase angular sweep (90 degrees)
    float phi_o = 0;        // Stance phase angular offset
    float t_c = 1.0;        // Gait period (1 second)
    float t_s = 0.5;        // Stance period (duty factor = 0.5)
    float t_d = 0;          // Turning offset (initially zero)
    
    // Initialize each leg's Buehler clock
    for (int i = 0; i < 6; i++) {
        // Time offset - phase shift between tripods
        float t_offset = 0;
        
        // Legs in TripodB are phase-shifted by half the gait period
        if (index_to_tripod_.at(i) == kTripodB) {
            t_offset = t_c / 2.0;
        }
        
        // Create a Buehler clock for this leg
        buehler_clocks_[i] = BuehlerClock(phi_s, phi_o, t_c, t_s, t_d, t_offset);
    }
    
    // Reset master time
    master_time_ = 0;
}

// Update the gait state based on elapsed time
void AlternatingTripod::update(float dt)
{
    // Update master time
    master_time_ += dt;
}

// Calculate the leg angles for all six legs
std::array<float, 6> AlternatingTripod::getLegAngles()
{
    std::array<float, 6> leg_angles;
    for (int i = 0; i < 6; i++) {
        // Get leg angle from corresponding Buehler clock
        leg_angles[i] = buehler_clocks_[i].getLegAngle(master_time_);
    }
    return leg_angles;
}

// Calculate the stance time based on desired velocity
float AlternatingTripod::getStanceTime(float des_velocity, float phi_s, float leg_length)
{
    if (std::abs(des_velocity) < 0.001) {
        // For very slow speeds, use a default stance time
        return 0.7;
    }
    
    // Calculate stance time based on velocity
    // t_s = (phi_s * r) / v
    // where phi_s is the stance phase angular sweep
    // r is the leg length (effective wheel radius)
    // v is the desired velocity
    float t_s_des = phi_s * leg_length / std::abs(des_velocity);
    
    // Clamp to reasonable values to ensure stability
    t_s_des = std::min(std::max(t_s_des, 0.1f), 0.9f);
    
    return t_s_des;
}

// Calculate turning parameter offsets for differential turning
std::pair<float, float> AlternatingTripod::getTurningParamOffsets(float des_angular)
{
    // Don't apply offsets if angular velocity is very small
    if (std::abs(des_angular) < 0.001) {
        return {0.0f, 0.0f};
    }
    
    // Calculate turning intensity (0.0 to 1.0) based on angular velocity
    float turn_intensity = std::min(std::abs(des_angular) / 2.0f, 1.0f);
    
    // Calculate stance time offset for turning
    // Positive = longer stance on one side, shorter on the other
    float delta_t_s = 0.2f * turn_intensity;
    
    // Calculate stance angle offset for turning
    // Shifts the center of the stance phase
    float delta_phi_o = 0.15f * turn_intensity;
    
    // Direction is based on desired angular velocity
    if (des_angular < 0) {
        delta_phi_o = -delta_phi_o;
    }
    
    return {delta_t_s, delta_phi_o};
}

// Update Buehler clock parameters from velocity command
std::tuple<float,float,float> AlternatingTripod::updateBuehlerFromVelocityCommand(float des_linear, float des_angular)
{
    // Calculate stance time based on desired linear velocity
    float t_s_des = getStanceTime(des_linear, M_PI/2, leg_length_);
    
    // Calculate turning parameter offsets
    auto [delta_t_s, delta_phi_o] = getTurningParamOffsets(des_angular);
    
    // Update each leg's Buehler clock parameters
    for (int i = 0; i < 6; i++) {
        // Base gait period and stance time
        buehler_clocks_[i].setGaitPeriod(1.0f);
        buehler_clocks_[i].setStancePeriod(t_s_des);
        
        // Apply turning offsets based on which side the leg is on
        if ((des_angular > 0 && index_to_side_.at(i) == kRight) ||
            (des_angular < 0 && index_to_side_.at(i) == kLeft)) {
            // Inside legs during turn: shorter stance time, shifted angle
            buehler_clocks_[i].setStancePeriod(t_s_des - delta_t_s);
            buehler_clocks_[i].setStanceOffset(delta_phi_o);
        }
        else if ((des_angular > 0 && index_to_side_.at(i) == kLeft) ||
                 (des_angular < 0 && index_to_side_.at(i) == kRight)) {
            // Outside legs during turn: longer stance time, shifted angle
            buehler_clocks_[i].setStancePeriod(t_s_des + delta_t_s);
            buehler_clocks_[i].setStanceOffset(-delta_phi_o);
        }
    }
    
    return std::make_tuple(t_s_des, delta_t_s, delta_phi_o);
}

// Command robot to stand
void AlternatingTripod::stand()
{
    // Reset all leg angles to standing position
    for (int i = 0; i < 6; i++) {
        // Set parameters for standing (all legs in the same position)
        buehler_clocks_[i].setStanceSweep(0);      // No leg swing
        buehler_clocks_[i].setStanceOffset(0);     // Centered stance
        buehler_clocks_[i].setGaitPeriod(1.0);     // Any period works for standing
        buehler_clocks_[i].setStancePeriod(1.0);   // 100% duty factor (always in stance)
        buehler_clocks_[i].setTurningOffset(0);    // No turning
        buehler_clocks_[i].setTimeOffset(0);       // No phase difference
    }
}

// Move specific legs to commanded positions
void AlternatingTripod::moveLegs(const std::vector<int>& leg_indices)
{
    // send motor commands to the legs here
}

// Wrap an angle to a specified range
float AlternatingTripod::angleWrap(float angle, int rotations_before_wrap, bool include_negatives)
{
    float wrap_value = rotations_before_wrap * 2 * M_PI;
    float wrapped_angle = std::fmod(angle, wrap_value);
    
    if (include_negatives) {
        // Allow negative angles up to -wrap_value/2
        if (wrapped_angle > wrap_value/2) {
            wrapped_angle -= wrap_value;
        } else if (wrapped_angle < -wrap_value/2) {
            wrapped_angle += wrap_value;
        }
    } else {
        // Force angles to be positive
        if (wrapped_angle < 0) {
            wrapped_angle += wrap_value;
        }
    }
    
    return wrapped_angle;
}

// Calculate the angular error between two angles
float AlternatingTripod::angularError(float angle1, float angle2, float direction_override)
{
    float error = angleWrap(angle1 - angle2, 1, true);
    
    // Apply direction override if provided
    if (direction_override > 0 && error < 0) {
        error += 2 * M_PI;
    } else if (direction_override < 0 && error > 0) {
        error -= 2 * M_PI;
    }
    
    return error;
} 