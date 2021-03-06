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
#include <visualization_msgs/MarkerArray.h>

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

  void endSimulation();

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
   * @return Quaternion as eigen vector (w, x, y, z)
   */
  Eigen::Quaterniond rpy2quaternion(double roll, double pitch, double yaw) const;

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
  void publishTransforms(const ros::Publisher &pub, const ros::Time time, const Eigen::Vector3d &position,
                         Eigen::Quaterniond attitude);
  void publishViewpoints(const ros::Publisher &pub, const Eigen::Vector3d &position,
                         const Eigen::Quaterniond &attitude);
  void publishPath(const ros::Publisher &pub, const std::vector<Eigen::Vector3d> path);

  /**
   * @brief Publish odometry of the viewpoint
   *
   * @param pub
   * @param position
   * @param attitude
   */
  void publishOdometry(const ros::Publisher &pub, const ros::Time time, const Eigen::Vector3d &position,
                       const Eigen::Quaterniond &attitude);
  void multiDOFJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg);
  geometry_msgs::Point toPoint(const Eigen::Vector3d &p);
  visualization_msgs::Marker viewpoint2MarkerMsg(int id, const Eigen::Vector3d &position,
                                                 const Eigen::Quaterniond &attitude,
                                                 Eigen::Vector3d color = Eigen::Vector3d(0.0, 0.0, 1.0));
  static Eigen::Matrix3d quat2RotMatrix(const Eigen::Vector4d &q) {
    Eigen::Matrix3d rotmat;
    rotmat << q(0) * q(0) + q(1) * q(1) - q(2) * q(2) - q(3) * q(3), 2 * q(1) * q(2) - 2 * q(0) * q(3),
        2 * q(0) * q(2) + 2 * q(1) * q(3),

        2 * q(0) * q(3) + 2 * q(1) * q(2), q(0) * q(0) - q(1) * q(1) + q(2) * q(2) - q(3) * q(3),
        2 * q(2) * q(3) - 2 * q(0) * q(1),

        2 * q(1) * q(3) - 2 * q(0) * q(2), 2 * q(0) * q(1) + 2 * q(2) * q(3),
        q(0) * q(0) - q(1) * q(1) - q(2) * q(2) + q(3) * q(3);
    return rotmat;
  };
  Eigen::Vector3d rayVector(int pixel_x, int pixel_y);
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  ros::Publisher image_pub;
  ros::Publisher depth_pub;
  ros::Publisher info_pub;
  ros::Publisher odometry_pub;
  ros::Publisher transforms_pub;
  ros::Publisher view_marker_pub;
  ros::Publisher path_pub;
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
   * @brief Attitude of vehicle
   *
   */
  Eigen::Quaterniond view_attitude_;

  Eigen::Quaterniond sensor_offset_{Eigen::Quaterniond(std::cos(0.5 * M_PI / 4), 0.0, -std::sin(0.5 * M_PI / 4), 0.0)};
  Eigen::Quaterniond blender_offset_{Eigen::Quaterniond(std::cos(0.5 * M_PI / 2), 0.0, 0.0, -std::sin(0.5 * M_PI / 2))};

  std::vector<visualization_msgs::Marker> viewpoint_marker_vector_;
  std::vector<Eigen::Vector3d> position_history_;
  bool new_viewpoint_{true};
  bool batch_created_{false};
  int image_width_ = 1440;
  int image_height_ = 1080;
};

#endif
