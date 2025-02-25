/**
 * @file buehler_clock.h
 * @brief Implements a Buehler Clock for legged robot gait generation
 * @version 0.1
 * @date 2024-11-17
 *
 * @copyright Copyright (c) 2024
 *
 */
#pragma once

#include <iostream>
#include <cmath>

class BuehlerClock
{
public:
  enum Mode
  {
    kSlow,
    kFastPre,
    kFastPost
  };
  
  struct BuehlerParams
  {
    float phi_s;    // Stance phase angular sweep
    float phi_o;    // Stance phase angular offset, 0 is centered around vertical
    float t_c;      // Gait Period
    float t_s;      // Stance period, duty cycle = t_s/t_c
    float t_d;      // Turning offset
    float t_offset; // Time offset for different legs
  };

  BuehlerClock(float phi_s, float phi_o, float t_c, float t_s, float t_d, float t_offset)
  {
    params_.phi_s = phi_s;
    params_.phi_o = phi_o;
    params_.t_c = t_c;
    params_.t_s = t_s;
    params_.t_d = t_d;
    params_.t_offset = t_offset;
  }

  BuehlerParams getParams() { return params_; };
  void setStanceSweep(float phi_s) { params_.phi_s = phi_s; }
  void setStanceOffset(float phi_o) { params_.phi_o = phi_o; }
  void setGaitPeriod(float t_c) { params_.t_c = t_c; }
  void setStancePeriod(float t_s) { params_.t_s = t_s; }
  void setTurningOffset(float t_d) { params_.t_d = t_d; }

  /**
   * @brief Calculate the Leg Angle
   *
   * @param time Current time
   * @param delta_t_turn Time offset for turning
   * @param delta_phi_turn Angle offset for turning
   * @return float Angle of the leg
   */
  float getLegAngle(float time, float delta_t_turn = 0, float delta_phi_turn = 0)
  {
    auto [phi_s, phi_o, t_c, t_s, t_d, t_offset] = params_;
    float t_phase = std::fmod(time, (params_.t_c - params_.t_offset));
    float output = 0;

    float negate_offset_multiplier = 1;
    Mode mode;
    //turning
    phi_o += delta_phi_turn;
    t_s += delta_t_turn;
    // Determine the mode
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

    switch (mode)
    {
    case Mode::kSlow:
      output = (phi_s / t_s) * t_phase - phi_o;
      break;
    case Mode::kFastPre:
      negate_offset_multiplier = -1;
    case Mode::kFastPost:
      output = phi_o +
               ((t_phase - negate_offset_multiplier * (t_c / 2.0)) * (2 * M_PI - phi_s) / (t_c - t_s));
      break;
    default:
      std::cout << "ERROR: BUEHLER CLOCK MODE NOT FOUND" << std::endl;
    }
    return output;
  }

private:
  BuehlerParams params_;
};