#include "oaisys_client/oaisys_planner.h"

#include <nav_msgs/Path.h>

OaisysPlanner::OaisysPlanner(const ros::NodeHandle &nh, const ros::NodeHandle &nh_private)
    : nh_(nh), nh_private_(nh_private) {
  image_pub = nh_.advertise<sensor_msgs::Image>("image", 1, true);
  depth_pub = nh_.advertise<sensor_msgs::Image>("depth", 1, true);
  info_pub = nh_.advertise<sensor_msgs::CameraInfo>("info", 1, true);
  transforms_pub = nh_.advertise<geometry_msgs::TransformStamped>("transform", 1, true);
  odometry_pub = nh_.advertise<nav_msgs::Odometry>("/ground_truth/odometry", 1, true);
  view_marker_pub = nh_.advertise<visualization_msgs::MarkerArray>("/viewpoints", 1, true);
  path_pub = nh_.advertise<nav_msgs::Path>("/path", 1, true);
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

  sensor_offset_ =
      Eigen::Quaterniond(mounting_rotation_w, mounting_rotation_x, mounting_rotation_y, mounting_rotation_z);
  sensor_offset_.normalize();

  oaisys_client_ = std::make_shared<OaisysClient>();

  // Publish initial odometry
  const Eigen::Vector3d origin(origin_x, origin_y, origin_z);
  view_position_ = origin;
  Eigen::Quaterniond view_attitude = rpy2quaternion(0.0, 0.0 / 180 * M_PI, 0.0);
  view_attitude_ = view_attitude;

  publishOdometry(odometry_pub, ros::Time::now(), origin, view_attitude);
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
    stepSample(view_position_, view_attitude_);
    new_viewpoint_ = false;
  } else {
    std::cout << "[OaisysPlanner] Waiting for viewpoint..." << std::endl;
  }
}

void OaisysPlanner::stepSample(const Eigen::Vector3d &position, const Eigen::Quaterniond &vehicle_attitude) {
  int batch_id, sample_id;

  Eigen::Quaterniond sensor_attitude = vehicle_attitude * sensor_offset_;

  Eigen::Vector3d blender_position(-position.x(), -position.y(), position.z());
  Eigen::Quaterniond blender_attitude(sensor_attitude.w(), -sensor_attitude.x(), -sensor_attitude.y(),
                                      sensor_attitude.z());
  blender_attitude = blender_attitude * blender_offset_;
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
    if (file_extension == "tif") exr_path = filepath;
    if (file_extension == "png") rgb_path = filepath;
  }

  /// TODO: Read
  // std::cout << "[OaisysPlanner]   - rgb file path: " << rgb_path << std::endl;
  // std::cout << "[OaisysPlanner]   - exr file path: " << exr_path << std::endl;
  cv::Mat rgb_image = cv::imread(rgb_path, cv::IMREAD_COLOR);
  cv::Mat depth_image = cv::imread(exr_path, cv::IMREAD_ANYDEPTH);
  depth_image = depth_image.mul(1.0/1.0855);
  // Threshold depth value so that freespace can be cleared up in voxblox
  cv::threshold(depth_image, depth_image, 51.0, -1, cv::ThresholdTypes::THRESH_TRUNC);

  ros::Time now_stamp = ros::Time::now();
  publishOdometry(odometry_pub, now_stamp, position, vehicle_attitude);
  publishTransforms(transforms_pub, now_stamp, position, sensor_attitude);
  publishCameraInfo(info_pub, now_stamp, position);
  PublishViewpointImage(image_pub, rgb_image, now_stamp, sensor_msgs::image_encodings::BGR8);
  PublishViewpointImage(depth_pub, depth_image, now_stamp, sensor_msgs::image_encodings::TYPE_32FC1);
  publishViewpoints(view_marker_pub, position, sensor_attitude);
  position_history_.push_back(position);
  publishPath(path_pub, position_history_);
}

void OaisysPlanner::publishPointClouds(const Eigen::Vector3d &position, const Eigen::Quaterniond &attitude,
                                       const std::string rgb_path, const std::string depth_path) {
  Eigen::Quaterniond sensor_attitude = attitude * rpy2quaternion(M_PI, -M_PI/2, 0.0);

  cv::Mat rgb_image = cv::imread(rgb_path, cv::IMREAD_COLOR);
  cv::Mat depth_image = cv::imread(depth_path, cv::IMREAD_ANYDEPTH);
  depth_image = depth_image.mul(1.0/1.0855);

  // std::cout.precision(17);
  std::cout << "[OaisysPlanner]    - depth: "<< depth_image.at<float>(511, 511) << std::endl;
  // Threshold depth value so that freespace can be cleared up in voxblox
  cv::threshold(depth_image, depth_image, 51.0, -1, cv::ThresholdTypes::THRESH_TRUNC);

  ros::Time now_stamp = ros::Time::now();
  publishOdometry(odometry_pub, now_stamp, position, sensor_attitude);
  publishTransforms(transforms_pub, now_stamp, position, sensor_attitude);
  publishCameraInfo(info_pub, now_stamp, position);
  PublishViewpointImage(image_pub, rgb_image, now_stamp, sensor_msgs::image_encodings::BGR8);
  PublishViewpointImage(depth_pub, depth_image, now_stamp, sensor_msgs::image_encodings::TYPE_32FC1);
  publishViewpoints(view_marker_pub, position, sensor_attitude);
  position_history_.push_back(position);
  publishPath(path_pub, position_history_);
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
  info_msg.P[2] = image_width_/2;
  info_msg.P[3] = 0.0;

  info_msg.P[4] = 0;
  info_msg.P[5] = 541.14;
  info_msg.P[6] = image_height_/2;
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

  Eigen::Quaterniond offset = rpy2quaternion(0.0, M_PI, M_PI / 2);
  attitude = attitude * offset;

  tf_transform.setOrigin(tf::Vector3(position.x(), position.y(), position.z()));
  tf::Quaternion q(attitude.x(), attitude.y(), attitude.z(), attitude.w());
  tf_transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(tf_transform, time, "map", "camera"));

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

void OaisysPlanner::publishOdometry(const ros::Publisher &pub, const ros::Time time, const Eigen::Vector3d &position,
                                    const Eigen::Quaterniond &attitude) {
  nav_msgs::Odometry msg;
  msg.header.stamp = time;
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
  view_attitude_ = Eigen::Quaterniond(
      msg.points.back().transforms.back().rotation.w, msg.points.back().transforms.back().rotation.x,
      msg.points.back().transforms.back().rotation.y, msg.points.back().transforms.back().rotation.z);
  /// TODO: This is a workaround with a wierd attitude offset
  view_attitude_ = view_attitude_ * Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0);
  new_viewpoint_ = true;
}

visualization_msgs::Marker OaisysPlanner::viewpoint2MarkerMsg(int id, const Eigen::Vector3d &position,
                                                              const Eigen::Quaterniond &attitude,
                                                              Eigen::Vector3d color) {
  double scale{1.5};  // Size of the viewpoint markers
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "my_namespace";
  marker.id = id;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  std::vector<geometry_msgs::Point> points;
  std::vector<Eigen::Vector3d> corner_ray_vectors;
  corner_ray_vectors.push_back(rayVector(0, 0));
  corner_ray_vectors.push_back(rayVector(0, image_width_));
  corner_ray_vectors.push_back(rayVector(image_height_, image_width_));
  corner_ray_vectors.push_back(rayVector(image_height_, 0));
  std::vector<Eigen::Vector3d> vertex;
  // void setOrientation(const Eigen::Vector4d &attitude) {
  //   orientation_ = attitude;

  //   // Transform corner ray vectors according to attitude
  //   for (auto &corner_ray : corner_ray_vectors_) {
  //   }
  //   center_ray_vector_ = R_att * center_ray_vector_;
  // }
  for (auto &corner_ray : corner_ray_vectors) {
    Eigen::Matrix3d R_att = quat2RotMatrix(Eigen::Vector4d(attitude.w(), attitude.x(), attitude.y(), attitude.z()));
    corner_ray = R_att * corner_ray;
    vertex.push_back(position + scale * corner_ray);
  }

  for (size_t i = 0; i < vertex.size(); i++) {
    points.push_back(toPoint(position));  // Viewpoint center
    points.push_back(toPoint(vertex[i]));
    points.push_back(toPoint(vertex[i]));
    points.push_back(toPoint(vertex[(i + 1) % vertex.size()]));
  }

  marker.points = points;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = 0.05;
  marker.scale.y = 0.0;
  marker.scale.z = 0.0;
  marker.color.a = 1.0;  // Don't forget to set the alpha!
  marker.color.r = color(0);
  marker.color.g = color(1);
  marker.color.b = color(2);
  return marker;
}

geometry_msgs::Point OaisysPlanner::toPoint(const Eigen::Vector3d &p) {
  geometry_msgs::Point position;
  position.x = p(0);
  position.y = p(1);
  position.z = p(2);
  return position;
}

Eigen::Vector3d OaisysPlanner::rayVector(int pixel_x, int pixel_y) {
  /// TODO: Get camera intrinsics from a file
  int c1 = image_height_ / 2;
  int c2 = image_width_ / 2;
  double f = image_width_ / 2;

  Eigen::Vector3d ray;
  ray << double(pixel_x - c1) / f, double(pixel_y - c2) / f, -1.0;
  ray.normalize();
  return ray;
}

void OaisysPlanner::publishViewpoints(const ros::Publisher &pub, const Eigen::Vector3d &position,
                                      const Eigen::Quaterniond &attitude) {
  visualization_msgs::MarkerArray msg;

  std::vector<visualization_msgs::Marker> marker;
  visualization_msgs::Marker mark;
  mark.action = visualization_msgs::Marker::DELETEALL;
  marker.push_back(mark);
  msg.markers = marker;
  pub.publish(msg);

  viewpoint_marker_vector_.insert(viewpoint_marker_vector_.begin(),
                                  viewpoint2MarkerMsg(viewpoint_marker_vector_.size(), position, attitude));
  msg.markers = viewpoint_marker_vector_;
  pub.publish(msg);
}

void OaisysPlanner::publishPath(const ros::Publisher &pub, const std::vector<Eigen::Vector3d> path) {
  nav_msgs::Path msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";
  std::vector<geometry_msgs::PoseStamped> poses;
  for (const auto &position : path) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.pose.position = toPoint(position);
    pose_stamped.pose.orientation.w = 1.0;
    pose_stamped.pose.orientation.x = 0.0;
    pose_stamped.pose.orientation.y = 0.0;
    pose_stamped.pose.orientation.z = 0.0;
    poses.push_back(pose_stamped);
  }
  msg.poses = poses;
  pub.publish(msg);
}

void OaisysPlanner::endSimulation() {
  bool finished{false};
  oaisys_client_->EndSimulation(finished);
  std::cout << "End simulation: " << finished << std::endl;
}
