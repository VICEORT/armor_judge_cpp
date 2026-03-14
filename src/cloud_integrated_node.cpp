#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <deque>
#include <memory>

class CloudIntegratorNode : public rclcpp::Node {
public:
  CloudIntegratorNode() : Node("cloud_integrator_node") {
    // 参数声明
    input_topic_  = declare_parameter<std::string>("input_topic", "/livox/lidar");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cloud_integrated");
    frame_id_     = declare_parameter<std::string>("frame_id", "livox_frame");
    window_sec_   = declare_parameter<double>("window_sec", 0.5);
    if (window_sec_ <= 0.0) window_sec_ = 0.5;

    auto sub_qos = rclcpp::SensorDataQoS();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

    sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_, sub_qos,
      std::bind(&CloudIntegratorNode::onCloud, this, std::placeholders::_1));
    pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(output_topic_, pub_qos);

    RCLCPP_INFO(get_logger(),
      "Started: input=%s output=%s frame=%s window=%.3f sec",
      input_topic_.c_str(), output_topic_.c_str(), frame_id_.c_str(), window_sec_);
  }

private:
  using PointT = pcl::PointXYZI;

  void onCloud(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    rclcpp::Time t_start = now();

    const rclcpp::Time t_msg = msg->header.stamp;

    // 转换为pcl点云
    pcl::PointCloud<PointT> cloud_in;
    try {
      pcl::fromROSMsg(*msg, cloud_in);
    } catch (const std::exception &e) {
      RCLCPP_ERROR(get_logger(), "pcl::fromROSMsg failed: %s", e.what());
      return;
    }

    // 时间回退处理
    if (!buffer_.empty() && t_msg < buffer_.back().first) {
      RCLCPP_WARN(get_logger(), "Time went backward, clearing buffer.");
      buffer_.clear();
    }

    // 加入缓冲区
    buffer_.emplace_back(t_msg, std::move(cloud_in));

    // 移除超出时间窗口的旧帧
    const rclcpp::Time newest = buffer_.back().first;
    while (!buffer_.empty() && (newest - buffer_.front().first).seconds() > window_sec_) {
      buffer_.pop_front();
    }

    // 计算总点数用于预分配
    size_t total_points = 0;
    for (const auto &entry : buffer_) {
      total_points += entry.second.size();
    }

    // 集成所有点云
    pcl::PointCloud<PointT> integrated;
    integrated.reserve(total_points);
    for (const auto &entry : buffer_) {
      integrated += entry.second;
    }

    // 发布
    sensor_msgs::msg::PointCloud2 out;
    pcl::toROSMsg(integrated, out);
    out.header.stamp = t_msg;
    out.header.frame_id = frame_id_;
    pub_->publish(out);

    rclcpp::Time t_end = now();
    RCLCPP_INFO(get_logger(),
      "Published integrated cloud: points=%zu msgs=%zu time_span=%.3f proc_time=%.3f ms",
      integrated.size(), buffer_.size(),
      (newest - buffer_.front().first).seconds(),
      (t_end - t_start).seconds() * 1000.0);
  }

  std::string input_topic_, output_topic_, frame_id_;
  double window_sec_;

  std::deque<std::pair<rclcpp::Time, pcl::PointCloud<PointT>>> buffer_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CloudIntegratorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}