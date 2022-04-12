/****************************************************************************
 *
 *   Copyright (c) 2022 Jaeyoung Lim. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/
/**
 * @brief Node to test mapping with oaisys client
 *
 *
 * @author Jaeyoung Lim <jalim@ethz.ch>
 */

#include "oaisys_client/oaisys_planner.h"

double getRandom(double min, double max) {
  return std::abs(max - min) * static_cast<double>(rand()) / static_cast<double>(RAND_MAX) + std::min(max, min);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "oaisys_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  auto oaisys_planner = std::make_shared<OaisysPlanner>(nh, nh_private);
  oaisys_planner->initialize(false);
  Eigen::Vector3d position{0.0, 0.0, 15.0};

  // Simulation Parameters
  size_t num_iters = 10;
  double max_angle = 0.5 * M_PI * 0.5;
  double theta = 0.0;

  for (size_t i = 0; i < num_iters; i++) {
    std::cout << "[OaisysMapping] Step Sample: " << i << std::endl;
    // double theta = getRandom(-max_angle, max_angle);
    // Eigen::Vector3d axis;
    // axis(0) = getRandom(-1.0, 1.0);
    // axis(1) = getRandom(-1.0, 1.0);
    // axis(2) = getRandom(-1.0, 1.0);
    // axis.normalize();
    // Eigen::Quaterniond attitude(std::cos(0.5 * theta), std::sin(0.5 * theta) * axis(0), std::sin(0.5 * theta) *
    // axis(1),
    //                     std::sin(0.5 * theta) * axis(2));
    position += Eigen::Vector3d(0.0, 0.0, 0.0);
    theta += M_PI / 6;
    Eigen::Vector3d axis(0.0, 0.0, 1.0);
    Eigen::Quaterniond attitude(std::cos(0.5 * theta), std::sin(0.5 * theta) * axis(0), std::sin(0.5 * theta) * axis(1),
                                std::sin(0.5 * theta) * axis(2));
    oaisys_planner->stepSample(position, attitude);
  }

  ros::spin();
  return 0;
}
