#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

namespace realsense_launch
{

class VoxelGridNode : public rclcpp::Node
{
public:
  explicit VoxelGridNode(const rclcpp::NodeOptions & options)
  : Node("voxel_grid", options)
  {
    // Declare parameters
    this->declare_parameter<double>("leaf_size", 0.01);
    this->declare_parameter<std::string>("filter_field_name", "");
    this->declare_parameter<double>("filter_limit_min", -std::numeric_limits<double>::max());
    this->declare_parameter<double>("filter_limit_max", std::numeric_limits<double>::max());
    this->declare_parameter<bool>("filter_limit_negative", false);
    this->declare_parameter<int>("queue_size", 10);

    // Get parameters
    leaf_size_ = this->get_parameter("leaf_size").as_double();
    filter_field_name_ = this->get_parameter("filter_field_name").as_string();
    filter_limit_min_ = this->get_parameter("filter_limit_min").as_double();
    filter_limit_max_ = this->get_parameter("filter_limit_max").as_double();
    filter_limit_negative_ = this->get_parameter("filter_limit_negative").as_bool();
    int queue_size = this->get_parameter("queue_size").as_int();

    // Create QoS profile for sensor data (BEST_EFFORT reliability for sensor data)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    qos.durability(rclcpp::DurabilityPolicy::Volatile);

    // Create subscriber and publisher with sensor data QoS
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "input",
      qos,
      std::bind(&VoxelGridNode::cloudCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("output", qos);

    RCLCPP_INFO(this->get_logger(), "VoxelGrid node initialized");
    RCLCPP_INFO(this->get_logger(), "  leaf_size: %.4f", leaf_size_);
    if (!filter_field_name_.empty()) {
      RCLCPP_INFO(this->get_logger(), "  filter_field: %s [%.2f, %.2f]",
                  filter_field_name_.c_str(), filter_limit_min_, filter_limit_max_);
    }
  }

private:
  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
  {
    try {
      // Convert ROS message to PCL point cloud
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      pcl::fromROSMsg(*cloud_msg, *cloud);

      // Apply voxel grid filter
      pcl::VoxelGrid<pcl::PointXYZRGB> voxel_filter;
      voxel_filter.setInputCloud(cloud);
      voxel_filter.setLeafSize(leaf_size_, leaf_size_, leaf_size_);

      // Apply field filter if specified
      if (!filter_field_name_.empty()) {
        voxel_filter.setFilterFieldName(filter_field_name_);
        voxel_filter.setFilterLimits(filter_limit_min_, filter_limit_max_);
        voxel_filter.setFilterLimitsNegative(filter_limit_negative_);
      }

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      voxel_filter.filter(*filtered_cloud);

      RCLCPP_DEBUG(this->get_logger(), "Filtered cloud: %zu -> %zu points",
                   cloud->points.size(), filtered_cloud->points.size());

      // Convert back to ROS message
      sensor_msgs::msg::PointCloud2 output_msg;
      pcl::toROSMsg(*filtered_cloud, output_msg);
      output_msg.header = cloud_msg->header;

      pub_->publish(output_msg);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error processing pointcloud: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  double leaf_size_;
  std::string filter_field_name_;
  double filter_limit_min_;
  double filter_limit_max_;
  bool filter_limit_negative_;
};

}  // namespace realsense_launch

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(realsense_launch::VoxelGridNode)
