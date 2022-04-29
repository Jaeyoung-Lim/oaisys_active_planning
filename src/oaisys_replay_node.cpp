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

#include <visualization_msgs/MarkerArray.h>
#include <fstream>

bool parsePoseFromFile(std::string text_path, std::vector<Eigen::Vector3d> &position,
                       std::vector<Eigen::Quaterniond> &attitude) {
  bool parse_result;

  std::ifstream file(text_path);
  std::string str;

  int id;
  // Look for the image file name in the path
  while (getline(file, str)) {
    std::stringstream ss(str);
    std::vector<std::string> data;
    std::string cc;
    while (getline(ss, cc, ',')) {
      data.push_back(cc);
    }
    // TX, TY, TZ, QW, QX, QY, QZ
    id = std::stoi(data[0]);
    position.push_back(Eigen::Vector3d(std::stof(data[1]), std::stof(data[2]), std::stof(data[3])));
    attitude.push_back(
        Eigen::Quaterniond(std::stof(data[4]), std::stof(data[5]), std::stof(data[6]), std::stof(data[7])));
    std::cout << "  - id: " << id << std::endl;
    std::cout << "    - position: " << position.back().transpose() << std::endl;
    // std::cout << "    - attitude: " << attitude.back() << std::endl;
  }

  return false;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "oaisys_planner");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_private("~");

  auto oaisys_planner = std::make_shared<OaisysPlanner>(nh, nh_private);

  std::string file_path, dir_path;
  nh_private.param<std::string>("file_path", file_path, file_path);
  nh_private.param<std::string>("image_dir", dir_path, dir_path);

  /// TODO: parse images from file

  /// TODO: parse poses from ste
  std::vector<Eigen::Vector3d> position;
  std::vector<Eigen::Quaterniond> attitude;
  std::cout << "[OaisysPlanner] file_path: " << file_path << std::endl;
  parsePoseFromFile(file_path, position, attitude);

  while (true) {
    for (size_t i = 0; i < position.size(); i++) {
      std::string rgb_path = dir_path + "/000" + std::to_string(i+2) + "sensor_left_rgb_00.png";
      std::string depth_path = dir_path + "/000" + std::to_string(i+2) + "sensor_left_pinhole_depth_00.exr";
      std::cout << "[OaisysPlanner]   - position   : " << position[i].transpose() << std::endl;
      // std::cout << "[OaisysPlanner]   - rgb_path   : " << rgb_path << std::endl;
      // std::cout << "[OaisysPlanner]   - depth_path : " << depth_path << std::endl;
      oaisys_planner->publishPointClouds(position[i], attitude[i], rgb_path, depth_path);
      ros::Duration(1.0).sleep();
    }
  }

  std::cout << "[OaisysPlanner] Done" << std::endl;
  ros::spin();
  return 0;
}
