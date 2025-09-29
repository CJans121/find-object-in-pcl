#include "find_object_in_pcl/find_object_in_pcl.hpp"

FindObjectInPcl::FindObjectInPcl(const std::string &name,
                                 const BT::NodeConfiguration &config,
                                 const rclcpp::Node::SharedPtr &ros_node)
    : BT::StatefulActionNode(name, config), ros_node_(ros_node) {
  // Subscribe to depth camera pointcloud
  pointcloud_sub_ =
      ros_node_->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/velodyne_points", 10,
          std::bind(&FindObjectInPcl::pointcloud_callback, this,
                    std::placeholders::_1));
}

BT::PortsList FindObjectInPcl::providedPorts() {
  return {BT::InputPort<std::string>("stl_path"),
	  BT::InputPort<std::string>("object_frame_name"),
          BT::InputPort<uint16_t>("timeout_secs"),
          BT::OutputPort<geometry_msgs::msg::TransformStamped>("pose")};
}
BT::NodeStatus FindObjectInPcl::onStart() {
  // Try getting the pointcloud
  if (!latest_cloud_) {
    RCLCPP_WARN(ros_node_->get_logger(), "No pointcloud received yet.");
    return BT::NodeStatus::FAILURE;
  }

  // Get STL path from input port
  std::string stl_path;
  if (!getInput("stl_path", stl_path)) {
    throw BT::RuntimeError(
        "FindObjectInPcl: missing required input [stl_path]");
  }

  // Get object frame name 
  if (!getInput("object_frame_name", object_frame_name_)) {
    throw BT::RuntimeError(
        "FindObjectInPcl: missing required input [object_frame_name]");
  }

  // Get timeout value from port
  if (!getInput("timeout_secs", timeout_secs_)) {
    RCLCPP_WARN(ros_node_->get_logger(),
                "No timeout specified, using default %u.",
                default_timeout_secs_);
    timeout_secs_ = default_timeout_secs_; // fallback
  }

  // Try loading the mesh
  pcl::PolygonMesh mesh;
  if (pcl::io::loadPolygonFileSTL(stl_path, mesh) == -1) {
    throw std::runtime_error("Could not load STL file: " + stl_path);
  }

  // Create a pointcloud of the object from the mesh and preprocess it
  pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_cloud);
  model_cloud_ = preprocess_cloud(mesh_cloud);

  // Record start time
  start_time_ = ros_node_->now();

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FindObjectInPcl::onRunning() {

  // Check for timeout
  rclcpp::Duration elapsed = ros_node_->now() - start_time_;
  if (elapsed.seconds() > timeout_secs_) {
    RCLCPP_WARN(ros_node_->get_logger(),
                "FindObjectInPcl timed out after %u seconds", timeout_secs_);
    return BT::NodeStatus::FAILURE;
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*latest_cloud_, *scene_cloud);

  // Preprocess scene
  pcl::PointCloud<pcl::PointXYZ>::Ptr scene_filtered =
      preprocess_cloud(scene_cloud);

  // ICP
  pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
  icp.setInputSource(model_cloud_);
  icp.setInputTarget(scene_filtered);

  // --- Registration tolerances ---
  icp.setMaxCorrespondenceDistance(max_correspondence_dist_);

  pcl::PointCloud<pcl::PointXYZ> aligned;
  icp.align(aligned);

  if (!icp.hasConverged()) {
    // Keep running until timeout
    return BT::NodeStatus::RUNNING;
  }

  // If ICP converged, return result
  Eigen::Matrix4f tf = icp.getFinalTransformation();

  geometry_msgs::msg::TransformStamped transform_msg;
  transform_msg.header.stamp = latest_cloud_->header.stamp;
  transform_msg.header.frame_id = latest_cloud_->header.frame_id;
  transform_msg.child_frame_id = object_frame_name_;

  transform_msg.transform.translation.x = tf(0, 3);
  transform_msg.transform.translation.y = tf(1, 3);
  transform_msg.transform.translation.z = tf(2, 3);

  Eigen::Quaternionf q(tf.block<3, 3>(0, 0));
  transform_msg.transform.rotation.x = q.x();
  transform_msg.transform.rotation.y = q.y();
  transform_msg.transform.rotation.z = q.z();
  transform_msg.transform.rotation.w = q.w();

  setOutput("pose", transform_msg);

  return BT::NodeStatus::SUCCESS;
}

void FindObjectInPcl::onHalted() {
  // Cleanup if necessary
}

void FindObjectInPcl::pointcloud_callback(
    const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  latest_cloud_ = msg;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr
FindObjectInPcl::preprocess_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr input) {
  // Downsample
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  pcl::PointCloud<pcl::PointXYZ>::Ptr down(new pcl::PointCloud<pcl::PointXYZ>);
  vg.setInputCloud(input);
  vg.setLeafSize(0.01f, 0.01f, 0.01f);
  vg.filter(*down);

  // Outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  sor.setInputCloud(down);
  sor.setMeanK(50);
  sor.setStddevMulThresh(1.0);
  sor.filter(*filtered);

  return filtered;
}
