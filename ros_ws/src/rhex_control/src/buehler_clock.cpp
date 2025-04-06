#include "buehler_clock.h"

// Constructor with default values
BuehlerClock::BuehlerClock(float phi_s, float phi_o, float t_c, float t_s, float t_d, float t_offset)
{
  params_.phi_s = phi_s;
  params_.phi_o = phi_o;
  params_.t_c = t_c;
  params_.t_s = t_s;
  params_.t_d = t_d;
  params_.t_offset = t_offset;
}

// Getters and setters
BuehlerClock::BuehlerParams BuehlerClock::getParams() 
{ 
  return params_; 
}

void BuehlerClock::setStanceSweep(float phi_s) 
{ 
  params_.phi_s = phi_s; 
}

void BuehlerClock::setStanceOffset(float phi_o) 
{ 
  params_.phi_o = phi_o; 
}

void BuehlerClock::setGaitPeriod(float t_c) 
{ 
  params_.t_c = t_c; 
}

void BuehlerClock::setStancePeriod(float t_s) 
{ 
  params_.t_s = t_s; 
}

void BuehlerClock::setTurningOffset(float t_d) 
{ 
  params_.t_d = t_d; 
}

// New method added for time offset
void BuehlerClock::setTimeOffset(float t_offset) 
{ 
  params_.t_offset = t_offset; 
}

// Calculate the leg angle based on the Buehler clock timing
float BuehlerClock::getLegAngle(float time, float delta_t_turn, float delta_phi_turn)
{
  // Extract params for readability
  auto [phi_s, phi_o, t_c, t_s, t_d, t_offset] = params_;
  
  // Apply timing offsets
  phi_o += delta_phi_turn;
  t_s += delta_t_turn;
  
  // Calculate phase time (time within the gait cycle)
  float t_phase = std::fmod(time + t_offset, t_c) - t_c/2;
  float output = 0;
  float negate_offset_multiplier = 1;
  Mode mode;

  // Determine the leg movement mode based on phase time
  if (t_phase < (-t_s / 2))
  {
    mode = kFastPre;
  }
  else if (t_phase < (t_s / 2))
  {
    mode = kSlow;
  }
  else
  {
    mode = kFastPost;
  }

  // Calculate the leg angle based on the mode
  switch (mode)
  {
  case Mode::kSlow:
    // Stance phase - leg moves slowly
    output = (phi_s / t_s) * t_phase - phi_o;
    break;
  case Mode::kFastPre:
    // Pre-stance swing phase - leg moves quickly
    negate_offset_multiplier = -1;
    // Intentional fall-through to the next case
  case Mode::kFastPost:
    // Post-stance swing phase - leg moves quickly
    output = phi_o +
             ((t_phase - negate_offset_multiplier * (t_c / 2.0)) * (2 * M_PI - phi_s) / (t_c - t_s));
    break;
  default:
    std::cout << "ERROR: BUEHLER CLOCK MODE NOT FOUND" << std::endl;
  }
  
  return output;
} 