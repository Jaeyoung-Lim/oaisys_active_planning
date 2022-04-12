#include "oaisys_client/oaisys_planner.h"

OaisysPlanner::OaisysPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  image_pub = nh_.advertise<sensor_msgs::Image>("image", 1, true);
  depth_pub = nh_.advertise<sensor_msgs::Image>("depth", 1, true);
  info_pub = nh_.advertise<sensor_msgs::CameraInfo>("info", 1, true);
  transforms_pub = nh_.advertise<geometry_msgs::TransformStamped>("transform", 1, true);
  odometry_pub = nh_.advertise<nav_msgs::Odometry>("/ground_truth/odometry", 1, true);
  viewpoint_sub = nh_.subscribe("/reference/command/trajectory", 1, &OaisysPlanner::multiDOFJointTrajectoryCallback,
                                this, ros::TransportHints().tcpNoDelay());

  double origin_x, origin_y;
  double origin_z{150.0};
  nh_private.param<double>("origin_x", origin_x, origin_x);
  nh_private.param<double>("origin_y", origin_y, origin_y);
  nh_private.param<double>("origin_z", origin_z, origin_z);
  double mounting_rotation_w{1.0};
  double mounting_rotation_x{0.0};
  double mounting_rotation_y{0.0};
  double mounting_rotation_z{0.0};

  nh_private.param<double>("mounting_rotation_w", mounting_rotation_w, mounting_rotation_w);
  nh_private.param<double>("mounting_rotation_x", mounting_rotation_x, mounting_rotation_x);
  nh_private.param<double>("mounting_rotation_y", mounting_rotation_y, mounting_rotation_y);
  nh_private.param<double>("mounting_rotation_z", mounting_rotation_z, mounting_rotation_z);
  sensor_offset_ << mounting_rotation_w, mounting_rotation_x, mounting_rotation_y, mounting_rotation_z;
  sensor_offset_.normalize();
  
  oaisys_client_ = std::make_shared<OaisysClient>();

  // Publish initial odometry
  const Eigen::Vector3d origin(origin_x, origin_y, origin_z);
  view_position_ = origin;
  Eigen::Vector3d vehicle_vel(15.0, 0.0, 0.0);
  view_attitude_ = rpy2quaternion(0.0, 0.0 / 180 * M_PI, 0.0);
  Eigen::Quaterniond view_attitude;
  view_attitude.w() = view_attitude_(0);
  view_attitude.x() = view_attitude_(1);
  view_attitude.y() = view_attitude_(2);
  view_attitude.z() = view_attitude_(3);
  publishOdometry(odometry_pub, origin, view_attitude);
}

void OaisysPlanner::initialize(bool spinner) {
  if (!oaisys_client_->StepBatch()) {
    std::cout << "[OaisysPlanner] StepBatch service failed" << std::endl;
  }
  /// TODO: This is a workaround so that oaisys doesn't crash. Not a good idea to sleep in the cosntructor!
  ros::Duration(5.0).sleep();

  if (!batch_created_) {
    int batch_finished{0};
    while (!oaisys_client_->BatchCreationFinished(batch_finished)) {
      std::cout << "[OaisysPlanner] Waiting for batch creation fished true: " << batch_finished << std::endl;
      ros::Duration(1.0).sleep();
    }
    std::cout << "[OaisysPlanner] BatchCreationFinished service result: " << batch_finished << std::endl;
    batch_created_ = true;
  }

  if (spinner) {
    double statusloop_dt_ = 2.0;
    ros::TimerOptions statuslooptimer_options(
        ros::Duration(statusloop_dt_), boost::bind(&OaisysPlanner::statusloopCallback, this, _1), &statusloop_queue_);
    statusloop_timer_ = nh_.createTimer(statuslooptimer_options);  // Define timer for constant loop rate

    statusloop_spinner_.reset(new ros::AsyncSpinner(1, &statusloop_queue_));
    statusloop_spinner_->start();
  }
}

void OaisysPlanner::statusloopCallback(const ros::TimerEvent &event) {
  if (new_viewpoint_) {
    const std::lock_guard<std::mutex> lock(viewpoint_mutex);
    std::cout << "[OaisysPlanner] Adding new viewpoint" << std::endl;
    Eigen::Quaterniond vehicle_attitude;
    vehicle_attitude.w() = view_attitude_(0);
    vehicle_attitude.x() = view_attitude_(1);
    vehicle_attitude.y() = view_attitude_(2);
    vehicle_attitude.z() = view_attitude_(3);
    stepSample(view_position_, vehicle_attitude);
    new_viewpoint_ = false;
  } else {
    std::cout << "[OaisysPlanner] Waiting for viewpoint..." << std::endl;
  }
}

void OaisysPlanner::stepSample(const Eigen::Vector3d &position, const Eigen::Quaterniond &vehicle_attitude) {
  int batch_id, sample_id;
  publishOdometry(odometry_pub, position, vehicle_attitude);

  Eigen::Quaterniond sensor_attitude;
  Eigen::Vector4d sensor_attitude_v = quatMultiplication(
      Eigen::Vector4d(vehicle_attitude.w(), vehicle_attitude.x(), vehicle_attitude.y(), vehicle_attitude.z()), sensor_offset_);
  sensor_attitude.w() = sensor_attitude_v(0);
  sensor_attitude.x() = sensor_attitude_v(1);
  sensor_attitude.y() = sensor_attitude_v(2);
  sensor_attitude.z() = sensor_attitude_v(3);

  Eigen::Vector3d blender_position(-position.x(), -position.y(), position.z());
  Eigen::Quaterniond blender_attitude(sensor_attitude.w(), -sensor_attitude.x(), -sensor_attitude.y(), sensor_attitude.z());

  oaisys_client_->StepSample(blender_position, blender_attitude, batch_id, sample_id);
  std::cout << "[OaisysPlanner] Running!: " << position.transpose() << std::endl;
  std::cout << "[OaisysPlanner]   - batch ID: " << batch_id << std::endl;
  std::cout << "[OaisysPlanner]   - sample ID: " << sample_id << std::endl;
  std::cout << "[OaisysPlanner]   - attitude: " << view_attitude_.w() << ", " << view_attitude_.x() << ", "
            << view_attitude_.y() << ", " << view_attitude_.z() << std::endl;
  std::cout << "[OaisysPlanner]   - position: " << position.x() << ", " << position.y() << ", " << position.z()
            << std::endl;
  std::cout << "[OaisysPlanner] Waiting for the render to be finished" << std::endl;
  bool render_finished{false};
  std::vector<std::string> filepath_list;
  while (!oaisys_client_->RenderFinished(render_finished, filepath_list)) {
    ros::Duration(1.0).sleep();
  }
  std::cout << "[OaisysPlanner] Render finished" << std::endl;
  std::string exr_path;
  std::string rgb_path;
  for (auto filepath : filepath_list) {
    std::string file_extension = filepath.substr(filepath.find_last_of(".") + 1);
    if (file_extension == "exr") exr_path = filepath;
    if (file_extension == "png") rgb_path = filepath;
  }
  std::cout << "[OaisysPlanner]   - rgb file path: " << rgb_path << std::endl;
  std::cout << "[OaisysPlanner]   - exr file path: " << exr_path << std::endl;
  cv::Mat rgb_image = cv::imread(rgb_path, cv::IMREAD_COLOR);
  cv::Mat depth_image = cv::imread(exr_path, cv::IMREAD_ANYDEPTH);

  ros::Time now_stamp = ros::Time::now();
  publishTransforms(transforms_pub, now_stamp, position, sensor_attitude);
  publishCameraInfo(info_pub, now_stamp, position);
  PublishViewpointImage(image_pub, rgb_image, now_stamp, sensor_msgs::image_encodings::BGR8);
  PublishViewpointImage(depth_pub, depth_image, now_stamp, sensor_msgs::image_encodings::TYPE_32FC1);
}

Eigen::Vector4d OaisysPlanner::rpy2quaternion(double roll, double pitch, double yaw) const {
  double cy = std::cos(yaw * 0.5);
  double sy = std::sin(yaw * 0.5);
  double cp = std::cos(pitch * 0.5);
  double sp = std::sin(pitch * 0.5);
  double cr = std::cos(roll * 0.5);
  double sr = std::sin(roll * 0.5);

  Eigen::Vector4d q;
  q(0) = cr * cp * cy + sr * sp * sy;
  q(1) = sr * cp * cy - cr * sp * sy;
  q(2) = cr * sp * cy + sr * cp * sy;
  q(3) = cr * cp * sy - sr * sp * cy;

  q.normalize();

  return q;
}

Eigen::Vector4d OaisysPlanner::quatMultiplication(const Eigen::Vector4d &q, const Eigen::Vector4d &p) const {
  Eigen::Vector4d quat;
  quat << p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3), p(0) * q(1) + p(1) * q(0) - p(2) * q(3) + p(3) * q(2),
      p(0) * q(2) + p(1) * q(3) + p(2) * q(0) - p(3) * q(1), p(0) * q(3) - p(1) * q(2) + p(2) * q(1) + p(3) * q(0);
  return quat;
}

void OaisysPlanner::PublishViewpointImage(const ros::Publisher &pub, const cv::Mat &image, const ros::Time time,
                                          const std::string encoding) {
  cv_bridge::CvImage msg;
  msg.header.stamp = time;
  msg.header.frame_id = "camera";
  msg.encoding = encoding;
  msg.image = image;
  sensor_msgs::Image image_msg = *msg.toImageMsg();
  pub.publish(image_msg);
}

void OaisysPlanner::publishCameraInfo(const ros::Publisher &pub, const ros::Time time,
                                      const Eigen::Vector3d &position) {
  sensor_msgs::CameraInfo info_msg;
  info_msg.header.stamp = time;
  info_msg.header.frame_id = "camera";
  info_msg.P[0] = 541.14;
  info_msg.P[1] = 0;
  info_msg.P[2] = 320;
  info_msg.P[3] = 0.0;

  info_msg.P[4] = 0;
  info_msg.P[5] = 541.14;
  info_msg.P[6] = 240;
  info_msg.P[7] = 0.0;

  info_msg.P[8] = 0;
  info_msg.P[9] = 0;
  info_msg.P[10] = 1;
  info_msg.P[11] = 0.0;

  pub.publish(info_msg);
}

void OaisysPlanner::publishTransforms(const ros::Publisher &pub, const ros::Time time, const Eigen::Vector3d &position,
                                      Eigen::Quaterniond attitude) {
  static tf::TransformBroadcaster br;
  tf::Transform tf_transform;

  Eigen::Vector4d offset = rpy2quaternion(0.0, M_PI, 0.0);
  Eigen::Vector4d transformed_attitude =
      quatMultiplication(Eigen::Vector4d(attitude.w(), attitude.x(), attitude.y(), attitude.z()), offset);
  attitude.w() = transformed_attitude(0);
  attitude.x() = transformed_attitude(1);
  attitude.y() = transformed_attitude(2);
  attitude.z() = transformed_attitude(3);

  tf_transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));
  tf::Quaternion q(attitude.x(), attitude.y(), attitude.z(), attitude.w());
  tf_transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(tf_transform, ros::Time::now(), "map", "camera"));

  geometry_msgs::TransformStamped msg;
  msg.header.stamp = time;
  msg.header.frame_id = "world";
  msg.child_frame_id = "camera";
  geometry_msgs::Transform transform;
  transform.translation.x = position.x();
  transform.translation.y = position.y();
  transform.translation.z = position.z();
  transform.rotation.x = attitude.x();
  transform.rotation.y = attitude.y();
  transform.rotation.z = attitude.z();
  transform.rotation.w = attitude.w();
  msg.transform = transform;
  pub.publish(msg);
}

void OaisysPlanner::publishOdometry(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                    Eigen::Quaterniond attitude) {
  nav_msgs::Odometry msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.child_frame_id = "camera";
  msg.pose.pose.position.x = position.x();
  msg.pose.pose.position.y = position.y();
  msg.pose.pose.position.z = position.z();
  msg.pose.pose.orientation.x = attitude.x();
  msg.pose.pose.orientation.y = attitude.y();
  msg.pose.pose.orientation.z = attitude.z();
  msg.pose.pose.orientation.w = attitude.w();
  pub.publish(msg);
}

void OaisysPlanner::multiDOFJointTrajectoryCallback(const trajectory_msgs::MultiDOFJointTrajectory &msg) {
  const std::lock_guard<std::mutex> lock(viewpoint_mutex);
  view_position_ << msg.points.back().transforms.back().translation.x,
      msg.points.back().transforms.back().translation.y, msg.points.back().transforms.back().translation.z;
  view_attitude_ << msg.points.back().transforms.back().rotation.w, msg.points.back().transforms.back().rotation.x,
      msg.points.back().transforms.back().rotation.y, msg.points.back().transforms.back().rotation.z;
  new_viewpoint_ = true;
}
