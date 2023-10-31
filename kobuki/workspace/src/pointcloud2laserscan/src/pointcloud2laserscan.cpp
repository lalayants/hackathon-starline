#define BOOST_BIND_NO_PLACEHOLDERS

#include <memory>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

#include <algorithm>
#include <functional>
#include <array>
#include <iostream>
#include <string_view>
// typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

using namespace std;
using namespace sensor_msgs;
using std::placeholders::_1;
using namespace std::chrono_literals;

std::vector<pair <float, float>> DATA;

#define PI 3.14159265  

class MinimalSubscriber : public rclcpp::Node
{
  public:
    MinimalSubscriber()
    : Node("minimal_subscriber")
    {
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>( 
      "/camera/depth/points", rclcpp::SensorDataQoS(), std::bind(&MinimalSubscriber::topic_callback, this, _1));
      publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan_2", 10);
      timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalSubscriber::timer_callback, this));
      
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) const
    {
      // RCLCPP_INFO(this->get_logger(), "I received the message , height is: '%d'", msg->height); //
      // std::cout << "Complete ROS cloud is: " << msg.get() << std::endl;
    //   printf ("Hello world!");
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
     
      pcl::fromROSMsg (*msg, *cloud);
      pcl::PointCloud<pcl::PointXYZ> cloud_new = *cloud;
     
      DATA.clear();
      float min_z = 1000000;
      float EPS = 0.05;
      float Z = 8.0;
      float height = 0.0;
      float min_height = 1000.0f;
      for (auto& point: cloud_new) {
          if (point.y < min_height) {
            min_height = point.y;
          }
      }
      min_height += 2.0;
      // RCLCPP_INFO(this->get_logger(), "min_height: '%f'", min_height);

      for (auto& point: cloud_new) {
        
        if (std::abs(point.y - 0) <= EPS) {
            // printf ("%f, %f, %f\n", point.x, point.y, point.z);
            DATA.push_back({point.x, point.z});
        }
      }
    }

    void timer_callback()
    {
      auto message = sensor_msgs::msg::LaserScan();
      std::vector<float> data;
      std::vector<float> data2;
      // std::cout << DATA.size() << std::endl;
      if (DATA.size() < 4) {
        return;
      }
      float delta = (PI / 3.0) / (float(DATA.size()) - 1);
      float all_count = delta*5.0;
      std::sort(DATA.begin(), DATA.end());
      // data.push_back(sqrt(DATA[0].first*DATA[0].first + DATA[0].second*DATA[0].second));
      float angle = delta;
      int i = 1;
      for (int i = DATA.size(); i >= 0; i--)
      {
          float x = DATA[i].first;
          float z = DATA[i].second;
          if (x <= 0.0) {
            printf ("%f, %f\n", x, z);
            data.push_back(sqrt(x*x + z*z));
            data2.push_back(0.0);
          }
      }
      for (int i = 0; i < (DATA.size() + 0)* 5; i++)
      {
        data.push_back(100.0);
        data2.push_back(0.0);
      }
      for (int i = DATA.size(); i >= 0; i--)
      {
          
          float x = DATA[i].first;
          float z = DATA[i].second;
          if (x > 0) {
            printf ("%f, %f\n", x, z);
            data.push_back(sqrt(x*x + z*z));
            data2.push_back(0.0);
          }
        
      }
    //   message.data = data
      message.intensities = data2;
      message.angle_min = 0.0;
      message.angle_max = 6.28000020980835;
      message.angle_increment = delta;
      message.range_min = 0.11999999731779099;
      message.range_max = 3.5;
      message.header.frame_id = "camera_link";
      message.ranges = data;
    //   RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    //   pc2_msg_->header.frame_id = "camera";
    //   publisher_->publish(*pc2_msg_);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    // sensor_msgs::msg::PointCloud2::SharedPtr pc2_msg_;
};

int main(int argc, char * argv[])
{
  std::cout << "Inside main function now";
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  // rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
