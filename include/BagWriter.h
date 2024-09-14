#pragma once

#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>

#include <eigen3/Eigen/Dense>
#include <queue>
#include <string>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_cpp/writers/sequential_writer.hpp>

template <typename T>
struct DataThread{
  std::queue<T> data_queue_;
  DataThread() {}

  void push(T data){
    data_queue_.push(data);
  }

  T pop(){
    T result;
    result = data_queue_.front();
    data_queue_.pop();
    return result;
  }

};

class BagWriter 
{
public:
    BagWriter(rclcpp::Node::SharedPtr& nh);
    ~BagWriter();
    std::string data_folder_path_;
    std::string sequence_name;
    std::string dst_path;

    void SaveRosbag();
    void ReadDataFromFile();
 
private:
    bool if_write_imu;
    bool if_write_gps;
    bool if_write_lidar;
    bool if_write_radar;
    bool if_write_gt;

    std::multimap<int64_t, std::string> data_stamp_;
    std::map<int64_t, sensor_msgs::msg::NavSatFix> gps_data_;
    std::map<int64_t, sensor_msgs::msg::Imu> imu_data_;

    DataThread<int64_t> data_stamp_thread_;
    DataThread<int64_t> gps_thread_;
    DataThread<int64_t> imu_thread_;
    DataThread<int64_t> radarpolar_thread_; 
    DataThread<int64_t> ouster_thread_;

    void loadDataStamp();
    void writeGps2bag(rosbag2_cpp::Writer& bag_writer);
    void writeImu2bag(rosbag2_cpp::Writer& bag_writer);
    void writeOuster2bag(rosbag2_cpp::Writer& bag_writer);
    void writeGt2bag(rosbag2_cpp::Writer& bag_writer);
    void writeRadar2bag(rosbag2_cpp::Writer& bag_writer);
    int GetDirList(std::string dir, std::vector<std::string> &files);    

    std::vector<std::string> ouster_file_list_;
    std::vector<std::string> radarpolar_file_list_;
    std::pair<std::string,sensor_msgs::msg::PointCloud2> ouster_next_;
    int search_bound_;
    int imu_data_version_;    
};