#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>

#include <sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include<boost/filesystem.hpp>
#include <fstream>

#include <ctime>
 

using namespace std;
namespace fs = boost::filesystem;

class Tokitti
{
	using PointT = pcl::PointXYZI;
public:
	Tokitti()
	{
	}
	
	~Tokitti()
	{
		of_points_stamp_.close();
		of_points_stamp_end.close();
		of_points_stamp_start_.close();
	}
	
	bool init()
	{
		ros::NodeHandle nh;
		ros::NodeHandle nh_private("~");
		
		nh_private.param<std::string>("pointclouds_topic",pointclouds_topic_,"/points");
		nh_private.param<std::string>("image_topic", image_topic_, "/image_raw");
		nh_private.param<std::string>("file_path",path_str_,"");
		
		if(path_str_.empty())
		{
			ROS_ERROR("file_path is empty!!");
			return false;
		}
		
		sub_point_cloud_ = nh.subscribe(pointclouds_topic_,0,&Tokitti::point_cloud_callback,this);
		sub_image_ = nh.subscribe(image_topic_,0,&Tokitti::image_callback, this);
		
		fs_path_ = fs::path(path_str_);
		points_path_str_ = path_str_ + "/velodyne_points";
		image_path_str_ = path_str_ + "/image_00";
		
		fs::create_directories(fs::path(points_path_str_+"/data"));
		fs::create_directories(fs::path(image_path_str_+"/data"));
		
		points_satrt_stamp_file_name_ = points_path_str_ + "/timestamps_start.txt";
		points_stamp_file_name_ = points_path_str_ + "/timestamps.txt";
		points_end_stamp_file_name_ = points_path_str_ + "/timestamps_end.txt";
		
		of_points_stamp_start_.open(points_satrt_stamp_file_name_);
		of_points_stamp_.open(points_stamp_file_name_);
		of_points_stamp_end.open(points_end_stamp_file_name_);
		
		image_stamp_file_name_ = image_path_str_ + "/timestamps.txt";
		of_image_stamp_.open(image_stamp_file_name_);
		
		return true;
	}
	
	void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
	{	
		static bool first_frame = true;
		static double start_time = 0;
		double end_time = msg->header.stamp.toSec();
		if(first_frame)
		{
			start_time = end_time;
			first_frame = false;
			return;
		}
	
		static int seq = 0;
		char file_name[10];
		sprintf(file_name,"%06d",seq++);
		string file = points_path_str_ + "/data/" + string(file_name) + ".bin";
		fstream output(file.c_str(), ios::out | ios::binary);
		
		if(!output.good())
		{
			cout  << "open file " << file << endl;
			return;
		}
		
		pcl::PointCloud<pcl::PointXYZI> cloud;
		pcl::fromROSMsg (*msg, cloud);
		
		//ROS_INFO("points size: %d", cloud.size());
		
		for(size_t i=0; i<cloud.size(); ++i)
		{
			output.write((char *)&(cloud[i]),4*sizeof(float));
		}
		output.close();
		
		of_points_stamp_start_ << timeConvert(start_time) << endl;
		of_points_stamp_ << timeConvert((start_time + end_time)/2) << endl;
		of_points_stamp_end << timeConvert(end_time) << endl;
		cout << timeConvert(start_time) << endl;
		start_time = end_time;
	}
	
	void image_callback(const sensor_msgs::Image::ConstPtr& msg)
	{
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::imshow("image", cv_ptr->image);
		cv::waitKey(1);
		
		static std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 100};
		static int cnt = 0;
		imwrite(image_path_str_ + "/data/" + std::to_string(cnt++)+".jpg", cv_ptr->image ,params);
	}
	
	string timeConvert(const double& timeStamp)
	{
		 stringstream ss;
		 time_t timeTemp;
		 ss << timeStamp;
		 ss >>  timeTemp;

		char temp[20];
		struct tm * timeSet = gmtime(&timeTemp);
		strftime(temp,50, "%Y-%m-%d %H:%M:%S", timeSet);
		return string(temp) + "." + to_string(int((timeStamp - int(timeStamp)) *1000000000));
	}


private:
	string path_str_;
	fs::path fs_path_;
	
	//point cloud
	std::string pointclouds_topic_;
	ros::Subscriber sub_point_cloud_;
	string points_path_str_;
	string points_stamp_file_name_;
	string points_satrt_stamp_file_name_;
	string points_end_stamp_file_name_;
	ofstream of_points_stamp_,
			 of_points_stamp_start_,
			 of_points_stamp_end;
	//image
	std::string image_topic_;
	ros::Subscriber sub_image_;
	string image_stamp_file_name_;
	string image_path_str_;
	ofstream of_image_stamp_;
};


int main(int argc, char** argv) 
{
	ros::init (argc, argv, "ros2kitti_node");
	Tokitti to_kitti;
	if(to_kitti.init())
		ros::spin();

	return 0;
}
