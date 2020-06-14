#include "ros2pcd.hpp"
#include <iomanip>


static void check_arguments(int argc, char* argv[])
{
  if (argc != 3){
    cout << "Please set arguments : save_dir topic_name'\n";
    exit(EXIT_FAILURE);
  }
}

void SavePCD::save_pcd(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  static int seq = 1;

  string file_name = save_path_ + std::to_string(seq) + ".pcd";
  pcl::PointCloud<pcl::PointXYZ> points;
  pcl::fromROSMsg(*msg, points);
  pcl::io::savePCDFileASCII(file_name, points);
  ROS_INFO("%2d:%s saved.",seq++, file_name.c_str());
}

void SavePCD::sub_pcd(int argc, char* argv[])
{
  ros::init(argc, argv, "PCD_Subscriber");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe(topic_name_, 1, &SavePCD::save_pcd, this);
  ros::spin();
}

int main(int argc, char* argv[])
{
  check_arguments(argc, argv);
  SavePCD saver;
  string path = argv[1];
  if (path[path.size() - 1] == '/')
    path.erase(path.begin() + path.size()-1);
    
  saver.save_path_ = path + '/';
  saver.topic_name_ = argv[2];
  saver.sub_pcd(argc, argv);

  return 0;
}

//rosrun ros2pcd ros2pcd_node save_path topic
