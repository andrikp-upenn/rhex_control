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

  BuehlerClock(float phi_s = M_PI/2, float phi_o = 0, float t_c = 1.0, float t_s = 0.5, float t_d = 0, float t_offset = 0);

  // Getters and setters for parameters
  BuehlerParams getParams();
  void setStanceSweep(float phi_s);
  void setStanceOffset(float phi_o);
  void setGaitPeriod(float t_c);
  void setStancePeriod(float t_s);
  void setTurningOffset(float t_d);
  void setTimeOffset(float t_offset);

  /**
   * @brief Calculate the Leg Angle
   *
   * @param time Current time
   * @param delta_t_turn Time offset for turning
   * @param delta_phi_turn Angle offset for turning
   * @return float Angle of the leg
   */
  float getLegAngle(float time, float delta_t_turn = 0, float delta_phi_turn = 0);

private:
  BuehlerParams params_;
};