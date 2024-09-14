# ros2bag_MulRan

ros2bag_MulRan is a ROS2 version of the data converter that converts [raw data from MulRan dataset](https://sites.google.com/view/mulran-pr/home) to ROS2 bag files modified from the [official file player respository](https://github.com/RPM-Robotics-Lab/file_player_mulran). More information can be found there.

## Dependencies
* Ubuntu 22.04
* ROS2 Humble
* PCL
* OpenCV
```
sudo apt install ros-humble-pcl*
sudo apt install libopencv-dev
```
## Compilation
```
cd ~/ros2_ws/src
git clone https://github.com/ASIG-X/ros2bag_MulRan
cd ..
colcon build --symlink-install
```
## Usage
1. Download the raw data from [MulRan dataset](https://sites.google.com/view/mulran-pr/home) and put them all under one folder.
2. Unzip Ouster data by running the following commands. 
```
tar -xvzf Ouster.tar.gz
tar xopf Ouster.tar
```
3. (Optional) Unzip radar data by running the following commands and set the parameter 'if_write_radar' in `launch/ros2bag_MulRan.launch.py` to 'True'.

Note: For the default parameter setting, radar data will not be written into bag files, and this step is unnecessary.
```
tar -xvzf polar.tar.gz
tar xopf polar.tar
```
4. Run the following command in the terminal 
```
# Optional: change parameters (e.g., set if_write_imu to false) in launch/bag_writer.launch.py for customized writing different sensor data to bag file.
cd ~/ros2_ws
source install/setup.bash
ros2 launch ros2bag_mulran ros2bag_MulRan.launch.py data_folder_path:=/path/to/mulran/data/folder/$squence_name sequence_name:=$squence_name dst_path:=/path/to/destination/folder
# e.g., ros2 launch ros2bag_mulran ros2bag_MulRan.launch.py data_folder_path:=/home/ziyu/Documents/KAIST01 sequence_name:=KAIST01 dst_path:=/home/ziyu/Documents/mulran
```
## Contributors
Ziyu Cao (email: ziyu.cao@liu.se)

Kailai Li (email: kailai.li@rug.nl)
## License
The source code is released under [GPLv3](https://www.gnu.org/licenses/) license.

We are constantly working on improving our code. For any technical issues, please contact Ziyu Cao (ziyu.cao@liu.se).
