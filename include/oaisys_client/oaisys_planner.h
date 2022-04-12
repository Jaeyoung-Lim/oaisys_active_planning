#ifndef OAISYS_PLANNER_H
#define OAISYS_PLANNER_H

#include "oaisys_client_ros/oaisys_client.h"

#include <ros/callback_queue.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <tf/transform_broadcaster.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>

#include <mutex>

class OaisysPlanner {
 public:
  OaisysPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private);
  ~OaisysPlanner(){};

  /**
   * @brief
   *
   * @param spinner Enable spinner
   */
  void initialize(bool spinner);
  /**
   * @brief Step sample
   *
   */
  void stepSample(const Eigen::Vector3d &position, const Eigen::Quaterniond &vehicle_attitude);

 private:
  /**
   * @brief Construct a new statusloop Callback object
   *
   * @param event
   */
  void statusloopCallback(const ros::TimerEvent &event);

  /**
   * @brief Convert roll pitch yaw to quaternions
   *
   * @param roll roll angle (rad)
   * @param pitch pitch angle (rad)
   * @param yaw yaw angle (rad)
   * @return Eigen::Vector4d Quaternion as eigen vector (w, x, y, z)
   */
  Eigen::Vector4d rpy2quaternion(double roll, double pitch, double yaw) const;

  /**
   * @brief Quaternion multiplication
   *
   * @param q
   * @param p
   * @return Eigen::Vector4d
   */
  Eigen::Vector4d quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) const;

  /**
   * @brief Publish cv::Mat image  into a ros topic
   *
   * @param pub ros publisher object
   * @param image image as a CV::Mat
   * @param time
   * @param encoding
   */
  void PublishViewpointImage(const ros::Publisher &pub, const cv::Mat &image, const ros::Time time,
                             const std::string encoding);
  void publishCameraInfo(const ros::Publisher &pub, const ros::Time time, const Eigen::Vector3d &position);
  void publishTransforms(const ros::Publisher &pub, const ros::Time time, const Eigen::Vector3d &position, Eigen::Quaterniond attitude);

  /**
   * @brief Publish odometry of the viewpoint
   *
   * @param pub
   * @param position
   * @param attitude
   */
  void publishOdometry(const ros::Publisher &pub, const Eigen::Vector3d &position, Eigen::Quaterniond attitude);
  void multiDOFJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher image_pub;
  ros::Publisher depth_pub;
  ros::Publisher info_pub;
  ros::Publisher odometry_pub;
  ros::Publisher transforms_pub;
  ros::Subscriber viewpoint_sub;

  ros::Timer statusloop_timer_;
  ros::CallbackQueue statusloop_queue_;

  std::unique_ptr<ros::AsyncSpinner> statusloop_spinner_;
  std::shared_ptr<OaisysClient> oaisys_client_;

  std::mutex viewpoint_mutex;
  /**
   * @brief Position of viewpoint center
   *
   */
  Eigen::Vector3d view_position_;

  /**
   * @brief Attitude of viewpoint
   *
   */
  Eigen::Vector4d view_attitude_;

  Eigen::Vector4d sensor_offset_{Eigen::Vector4d(std::cos(0.5 * M_PI / 4), 0.0, -std::sin(0.5 * M_PI / 4), 0.0)};
  bool new_viewpoint_{true};
  bool batch_created_{false};
};

#endif
