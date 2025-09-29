#ifndef FIND_OBJECT_IN_PCL_HPP_
#define FIND_OBJECT_IN_PCL_HPP_

#include <behaviortree_cpp/action_node.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <rclcpp/rclcpp.hpp>


#include <pcl/point_cloud.h>// Core PCL point cloud container type
#include <pcl/point_types.h>// Definitions for point types (e.g., pcl::PointXYZ, pcl::PointNormal, etc.)
#include <pcl/io/vtk_lib_io.h>// For loading STL / mesh files into PCL (loadPolygonFileSTL, etc.)
#include <pcl/registration/icp.h>// ICP (Iterative Closest Point) algorithm for pointcloud alignment
#include <pcl_conversions/pcl_conversions.h>// Conversions between ROS2 sensor_msgs/PointCloud2 <-> PCL point clouds
#include <pcl/filters/voxel_grid.h>// Downsampling pointclouds using voxel grid filter
#include <pcl/filters/statistical_outlier_removal.h>// Removing statistical outliers (noise filtering)

class FindObjectInPcl : public BT::StatefulActionNode
{
public:
    FindObjectInPcl(const std::string& name,
                    const BT::NodeConfiguration& config,
                    const rclcpp::Node::SharedPtr& ros_node);

    static BT::PortsList providedPorts();

    BT::NodeStatus onStart() override;
    BT::NodeStatus onRunning() override;
    void onHalted() override;

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocess_cloud(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input);

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    sensor_msgs::msg::PointCloud2::SharedPtr latest_cloud_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_;
    double default_timeout_secs_ = 10.0;
};

#endif  // FIND_OBJECT_IN_PCL_HPP_
