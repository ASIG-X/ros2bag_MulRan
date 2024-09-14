#include "../include/BagWriter.h"
#include "boost/lexical_cast.hpp" 

struct PointXYZIRT {
  PCL_ADD_POINT4D;
  float intensity;
  uint32_t t;
  int ring;

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}EIGEN_ALIGN16;

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
                                   (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
                                   (uint32_t, t, t) (int, ring, ring)
                                   )

BagWriter::BagWriter(rclcpp::Node::SharedPtr& nh)
{
  search_bound_ = 10;
  imu_data_version_ = 0;

  nh->declare_parameter<std::string>("data_folder_path");
  nh->get_parameter("data_folder_path", data_folder_path_);  
  
  nh->declare_parameter<std::string>("sequence_name");
  nh->get_parameter("sequence_name", sequence_name);    
  
  nh->declare_parameter<std::string>("dst_path");
  nh->get_parameter("dst_path", dst_path);      

  nh->declare_parameter<bool>("if_write_imu");
  nh->get_parameter_or("if_write_imu", if_write_imu, true);    

  nh->declare_parameter<bool>("if_write_gps");
  nh->get_parameter_or("if_write_gps", if_write_gps, true);      

  nh->declare_parameter<bool>("if_write_lidar");
  nh->get_parameter_or("if_write_lidar", if_write_lidar, true);    

  nh->declare_parameter<bool>("if_write_radar");
  nh->get_parameter_or("if_write_radar", if_write_radar, true);    

  nh->declare_parameter<bool>("if_write_gt");
  nh->get_parameter_or("if_write_gt", if_write_gt, true);          
}


BagWriter::~BagWriter() { }

void BagWriter::ReadDataFromFile()
{
  std::ifstream f((data_folder_path_+"/data_stamp.csv").c_str());
  if(!f.good()){
    std::cout << "Please check the file path. The input path is wrong (data_stamp.csv not exist)" << std::endl;
    return;
  }
  f.close();

  FILE *fp;
  int64_t stamp;

  fp = fopen((data_folder_path_+"/data_stamp.csv").c_str(),"r");
  char data_name[50];
  data_stamp_.clear();
  while(fscanf(fp,"%ld,%s\n",&stamp,data_name) == 2){
    data_stamp_.insert( std::multimap<int64_t, std::string>::value_type(stamp, data_name));
  }
  std::cout << "Stamp data are loaded" << std::endl;
  fclose(fp);

  if (if_write_gps) {
    fp = fopen((data_folder_path_+"/gps.csv").c_str(),"r");
    double latitude, longitude, altitude;
    double cov[9];
    sensor_msgs::msg::NavSatFix gps_data;
    gps_data_.clear();
    while( fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                  &stamp,&latitude,&longitude,&altitude,&cov[0],&cov[1],&cov[2],&cov[3],&cov[4],&cov[5],&cov[6],&cov[7],&cov[8])
          == 13
          )
    {
      gps_data.header.stamp = rclcpp::Time(stamp);
      gps_data.header.frame_id = "gps";
      gps_data.latitude = latitude;
      gps_data.longitude = longitude;
      gps_data.altitude = altitude;
      for(int i = 0 ; i < 9 ; i ++) gps_data.position_covariance[i] = cov[i];
      gps_data_[stamp] = gps_data;
    }
    std::cout << "Gps data are loaded" << std::endl;

    fclose(fp);
  }

  if(if_write_imu) {
    fp = fopen((data_folder_path_+"/xsens_imu.csv").c_str(),"r");
    double q_x,q_y,q_z,q_w,x,y,z,g_x,g_y,g_z,a_x,a_y,a_z,m_x,m_y,m_z;
    sensor_msgs::msg::Imu imu_data;
    imu_data_.clear();
    while(1) {
      int length = fscanf(fp,"%ld,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n", \
                          &stamp,&q_x,&q_y,&q_z,&q_w,&x,&y,&z,&g_x,&g_y,&g_z,&a_x,&a_y,&a_z,&m_x,&m_y,&m_z);
      if(length != 8 && length != 17)
        break;
      if(length == 8) {
        imu_data.header.stamp = rclcpp::Time(stamp);
        imu_data.header.frame_id = "imu";
        imu_data.orientation.x = q_x;
        imu_data.orientation.y = q_y;
        imu_data.orientation.z = q_z;
        imu_data.orientation.w = q_w;

        imu_data_[stamp] = imu_data;
        imu_data_version_ = 1;
      } else if(length == 17) {
        imu_data.header.stamp = rclcpp::Time(stamp);
        imu_data.header.frame_id = "imu";
        imu_data.orientation.x = q_x;
        imu_data.orientation.y = q_y;
        imu_data.orientation.z = q_z;
        imu_data.orientation.w = q_w;
        imu_data.angular_velocity.x = g_x;
        imu_data.angular_velocity.y = g_y;
        imu_data.angular_velocity.z = g_z;
        imu_data.linear_acceleration.x = a_x;
        imu_data.linear_acceleration.y = a_y;
        imu_data.linear_acceleration.z = a_z;

        imu_data.orientation_covariance[0] = 3;
        imu_data.orientation_covariance[4] = 3;
        imu_data.orientation_covariance[8] = 3;
        imu_data.angular_velocity_covariance[0] = 3;
        imu_data.angular_velocity_covariance[4] = 3;
        imu_data.angular_velocity_covariance[8] = 3;
        imu_data.linear_acceleration_covariance[0] = 3;
        imu_data.linear_acceleration_covariance[4] = 3;
        imu_data.linear_acceleration_covariance[8] = 3;
        imu_data_[stamp] = imu_data;

        imu_data_version_ = 2;
      }
    }
    std::cout << "IMU data are loaded" << std::endl;
    std::fclose(fp);
  } 

  ouster_file_list_.clear();
  radarpolar_file_list_.clear();

  if (if_write_lidar) {
    GetDirList(data_folder_path_ + "/Ouster", ouster_file_list_);
  }
  if (if_write_radar) {
    GetDirList(data_folder_path_ + "/polar", radarpolar_file_list_);
  }
  loadDataStamp();
}

void BagWriter::loadDataStamp()
{
  for(auto iter = data_stamp_.begin() ; iter != data_stamp_.end() ; iter ++) {
    auto stamp = iter->first;
    if(iter->second.compare("imu") == 0) {
      imu_thread_.push(stamp);
    } else if(iter->second.compare("gps") == 0) {
      gps_thread_.push(stamp);
    } else if(iter->second.compare("ouster") == 0) {
      ouster_thread_.push(stamp);
    } else if(iter->second.compare("radar") == 0) {
      radarpolar_thread_.push(stamp);
    }
  }
  std::cout << "Data publish complete. loaded imu: " << imu_thread_.data_queue_.size()
            << " loaded gps: " << gps_thread_.data_queue_.size()
            << " loaded ouster: " << ouster_thread_.data_queue_.size()
            << " loaded radar: " << radarpolar_thread_.data_queue_.size() << std::endl;
}

void BagWriter::writeGps2bag(rosbag2_cpp::Writer& bag_writer)
{
  while(!gps_thread_.data_queue_.empty()){
    auto data = gps_thread_.pop();
    if(gps_data_.find(data) != gps_data_.end()){
      sensor_msgs::msg::NavSatFix gps_msg = gps_data_[data];
      bag_writer.write(gps_msg, "/gps/fix", rclcpp::Time(gps_msg.header.stamp));      
    }

  }
}

void BagWriter::writeImu2bag(rosbag2_cpp::Writer& bag_writer)
{
  while(!imu_thread_.data_queue_.empty()) {
    auto data = imu_thread_.pop();
    if(imu_data_.find(data) != imu_data_.end()) {
      sensor_msgs::msg::Imu imu_msg = imu_data_[data];
      bag_writer.write(imu_msg, "/imu/data_raw", rclcpp::Time(imu_msg.header.stamp));
    }
  }  
}

void BagWriter::writeOuster2bag(rosbag2_cpp::Writer& bag_writer)
{
  int current_file_index = 0;
  int previous_file_index = 0;

    while(!ouster_thread_.data_queue_.empty()) {
      auto data = ouster_thread_.pop();

      if(std::to_string(data) + ".bin" == ouster_next_.first) {
        ouster_next_.second.header.stamp = rclcpp::Time(data);
        ouster_next_.second.header.frame_id = "ouster"; // frame ID
        bag_writer.write(ouster_next_.second, "/os1_points", rclcpp::Time(data));
      } else {
        pcl::PointCloud<PointXYZIRT> cloud;
        cloud.clear();
        sensor_msgs::msg::PointCloud2 publish_cloud;
        std::string current_file_name = data_folder_path_ + "/Ouster" +"/"+ std::to_string(data) + ".bin";

        if(std::find(std::next(ouster_file_list_.begin(),std::max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),std::to_string(data)+".bin") != ouster_file_list_.end())
        {
          std::ifstream file;
          file.open(current_file_name, std::ios::in|std::ios::binary);
          int k = 0;
          while(!file.eof())
          {
            PointXYZIRT point;
            file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
            file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
            point.ring = (k%64);
            k = k+1 ;
            cloud.points.push_back (point);
          }
          file.close();

          pcl::toROSMsg(cloud, publish_cloud);
          publish_cloud.header.stamp = rclcpp::Time(data);
          publish_cloud.header.frame_id = "ouster";
          bag_writer.write(publish_cloud, "/os1_points", rclcpp::Time(data));
        }
        previous_file_index = 0;
      }

      //load next data
      pcl::PointCloud<PointXYZIRT> cloud;
      cloud.clear();
      sensor_msgs::msg::PointCloud2 publish_cloud;
      current_file_index = std::find(std::next(ouster_file_list_.begin(),std::max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),std::to_string(data)+".bin") - ouster_file_list_.begin();
      if(std::find(std::next(ouster_file_list_.begin(),std::max(0,previous_file_index-search_bound_)),ouster_file_list_.end(),ouster_file_list_[current_file_index+1]) != ouster_file_list_.end()){
        std::string next_file_name = data_folder_path_ + "/Ouster" +"/"+ ouster_file_list_[current_file_index+1];

        std::ifstream file;
        file.open(next_file_name, std::ios::in|std::ios::binary);
        int k = 0;
        while(!file.eof()){
          PointXYZIRT point;
          file.read(reinterpret_cast<char *>(&point.x), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.y), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.z), sizeof(float));
          file.read(reinterpret_cast<char *>(&point.intensity), sizeof(float));
          point.ring = (k%64);
          k = k+1 ;
          cloud.points.push_back (point);
        }

        file.close();
        pcl::toROSMsg(cloud, publish_cloud);
        ouster_next_ = make_pair(ouster_file_list_[current_file_index+1], publish_cloud);
      }
      previous_file_index = current_file_index;
    }
}

void BagWriter::writeGt2bag(rosbag2_cpp::Writer& bag_writer)
{
  const std::string gt_csv_path = data_folder_path_+ std::string("/global_pose.csv");
  std::fstream fin;
  fin.open(gt_csv_path, std::ios::in);
  if(fin.is_open()){
    std::cout<<"loaded: "<<gt_csv_path<< std::endl;

    rclcpp::SerializedMessage serialized_msg;

    std::string temp;
    geometry_msgs::msg::PoseStamped gt_msg;
    gt_msg.header.frame_id = "world";
    while (fin >> temp) {
      Eigen::Matrix<double,4,4> T = Eigen::Matrix<double,4,4>::Zero();
      T(3,3) = 1.0;

      std::vector<std::string> row;

      std::stringstream  ss(temp);
      std::string str;
      while (getline(ss, str, ','))
        row.push_back(str);
      if(row.size()!=13)
        break;
      int64_t stamp_int;
      std::istringstream ( row[0] ) >> stamp_int;
      for(int i=0;i<3;i++){
        for(int j=0;j<4;j++){
          double d = boost::lexical_cast<double> (row[1+(4*i)+j]);
          T(i,j) = d;
        }
      }

      Eigen::Quaterniond ort(T.topLeftCorner<3, 3>());

      gt_msg.pose.position.x = T(0, 3);
      gt_msg.pose.position.y = T(1, 3);
      gt_msg.pose.position.z = T(2, 3);
      gt_msg.pose.orientation.w = ort.w();
      gt_msg.pose.orientation.x = ort.x();
      gt_msg.pose.orientation.y = ort.y();
      gt_msg.pose.orientation.z = ort.z();
      
      gt_msg.header.stamp = rclcpp::Time(stamp_int);
      bag_writer.write(gt_msg, "/gt", rclcpp::Time(stamp_int));
    }
  }  
}

void BagWriter::writeRadar2bag(rosbag2_cpp::Writer& bag_writer)
{
  GetDirList(data_folder_path_ + "/polar", radarpolar_file_list_);
  std::cout<<"Found: "<<radarpolar_file_list_.size()<<" radar sweeps"<< std::endl;
  for(auto && file_name : radarpolar_file_list_){
    const std::string file_path = data_folder_path_ + "/polar/" + file_name;

    size_t lastindex = file_name.find_last_of(".");
    std::string stamp_str = file_name.substr(0, lastindex);
    int64_t  stamp_int;
    std::istringstream ( stamp_str ) >> stamp_int;

    cv_bridge::CvImage radarpolar_out_msg;
    radarpolar_out_msg.header.stamp = rclcpp::Time(stamp_int);
    radarpolar_out_msg.header.frame_id = "radar_polar";
    radarpolar_out_msg.encoding = sensor_msgs::image_encodings::MONO8;
    radarpolar_out_msg.image    = cv::imread(file_path, 0);
    sensor_msgs::msg::Image::SharedPtr msg = radarpolar_out_msg.toImageMsg();
    bag_writer.write(*msg, "/radar/polar", rclcpp::Time(stamp_int));
  }
}

int BagWriter::GetDirList(std::string dir, std::vector<std::string> &files)
{
  files.clear();
  std::vector<std::string> tmp_files;
  struct dirent **namelist;
  int n;
  n = scandir(dir.c_str(),&namelist, 0 , alphasort);
  if (n < 0)
  {
    std::string errmsg{(std::string{"No directory ("} + dir + std::string{")"})};
    const char * ptr_errmsg = errmsg.c_str();
    perror(ptr_errmsg);
  } else {
    while (n--) {
      if(std::string(namelist[n]->d_name) != "." && std::string(namelist[n]->d_name) != "..")
      {
        tmp_files.push_back(std::string(namelist[n]->d_name));
      }
      free(namelist[n]);
    }
    free(namelist);
  }

  for(auto iter = tmp_files.rbegin() ; iter!= tmp_files.rend() ; iter++) {
    files.push_back(*iter);
  }
  return 0;
}

void BagWriter::SaveRosbag()
{
  const std::string bag_path = dst_path +"/" + sequence_name;
  std::cout<<"Storing bag to: "<<bag_path<< std::endl;

  auto rosbag_directory = rcpputils::fs::path(bag_path);
  rcpputils::fs::remove_all(rosbag_directory);

  rosbag2_cpp::Writer writer;
  rosbag2_storage::StorageOptions storage_options;
  // storage_options.storage_id = GetParam();
  storage_options.uri = rosbag_directory.string();
  writer.open(storage_options);

  if (if_write_imu) {
    std::cout<<"writing imu msg to bag..."<< std::endl;
    writeImu2bag(writer);
  }

  if (if_write_gps) {
    std::cout<<"writing gps msg to bag..."<< std::endl;
    writeGps2bag(writer);
  }

  if (if_write_lidar) {
    std::cout<<"writing lidar msg to bag..."<< std::endl;
    writeOuster2bag(writer);
  }

  if (if_write_radar) {
    std::cout<<"writing radar msg to bag..."<< std::endl;
    writeRadar2bag(writer);
  }

  if (if_write_gt) {
    std::cout<<"writing gt msg to bag..."<< std::endl;
    writeGt2bag(writer);
  }

  std::cout<<"rosbag stored at: "<<bag_path<< std::endl;
}
