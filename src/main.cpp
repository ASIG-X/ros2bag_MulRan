#include "BagWriter.h"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto nh = rclcpp::Node::make_shared("ros2bag_mulran");
  BagWriter bag_writer(nh);
  bag_writer.ReadDataFromFile();
  bag_writer.SaveRosbag();
  rclcpp::shutdown();
}
