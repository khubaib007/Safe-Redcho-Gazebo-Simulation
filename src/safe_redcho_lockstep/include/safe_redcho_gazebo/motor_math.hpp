#pragma once
#include <algorithm>
#include <array>
#include <cmath>

namespace safe_redcho_gazebo
{
struct MotorWrench
{
  std::array<double, 4> motors{};
  double thrust{0.0};
  std::array<double, 3> torque{};
};
inline MotorWrench MixMotors(const std::array<double, 4> &command,
                            double limit, double lever, double torqueRatio)
{
  MotorWrench out;
  for (std::size_t i=0; i<4; ++i)
    out.motors[i] = std::clamp(command[i], 0.0, limit);
  const auto &f = out.motors;
  out.thrust = f[0]+f[1]+f[2]+f[3];
  out.torque = {lever*(-f[0]-f[1]+f[2]+f[3]),
                lever*(-f[0]+f[1]+f[2]-f[3]),
                torqueRatio*(-f[0]+f[1]-f[2]+f[3])};
  return out;
}
}
