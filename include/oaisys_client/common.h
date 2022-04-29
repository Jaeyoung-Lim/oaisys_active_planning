#ifndef OAISYS_CLIENT_COMMON_H
#define OAISYS_CLIENT_COMMON_H

#include <Eigen/Dense>

/**
 * @brief Convert roll pitch yaw to quaternions
 *
 * @param roll roll angle (rad)
 * @param pitch pitch angle (rad)
 * @param yaw yaw angle (rad)
 * @return Quaternion as eigen vector (w, x, y, z)
 */
Eigen::Quaterniond rpy2quaternion(double roll, double pitch, double yaw) {
  Eigen::Quaterniond q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) *
                         Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                         Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
  return q;
}
#endif
