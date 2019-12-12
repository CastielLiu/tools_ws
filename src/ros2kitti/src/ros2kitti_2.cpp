#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <sensor_msgs/Image.h>
#include<cv_bridge/cv_bridge.h>
#include<opencv2/opencv.hpp>
#include<opencv2/highgui/highgui.hpp>
#include<opencv2/imgproc/imgproc.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <boost/filesystem.hpp>
#include <memory>
#include <fstream>

#include <ctime>
 

using namespace std;
namespace fs = boost::filesystem;

class Tokitti
{
	using PointT = pcl::PointXYZI;
	typedef message_filters::sync_policies::ApproximateTime
			<sensor_msgs::PointCloud2,sensor_msgs::Image>  MySyncPolicy;
	typedef message_filters::Synchronizer<MySyncPolicy> Suychronizer;
	typedef message_filters::Subscriber<sensor_msgs::PointCloud2> SubPointCloud2;
	typedef message_filters::Subscriber<sensor_msgs::Image> SubImage;
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
		
		sub_point_cloud_.reset(new SubPointCloud2(nh, pointclouds_topic_,5));
		sub_image_.reset(new SubImage(nh, image_topic_, 5));
		sync_.reset(new Suychronizer(MySyncPolicy(10), *sub_point_cloud_, *sub_image_));
		sync_->registerCallback(boost::bind(&Tokitti::callback,this,_1,_2));
		
		path_str_ = createDataDir(path_str_);
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
	
	void callback(const sensor_msgs::PointCloud2::ConstPtr& points_msg,
				  const sensor_msgs::Image::ConstPtr& image_msg)
	{
		static bool first_frame = true;
		static double start_time = 0;
		double end_time = points_msg->header.stamp.toSec();
		if(first_frame)
		{
			start_time = end_time;
			first_frame = false;
			return;
		}
		/*  save points start       */
		static int seq = 0;
		char seq_str[10];
		sprintf(seq_str,"%06d",seq++);
		string file = points_path_str_ + "/data/" + string(seq_str) + ".bin";
		fstream output(file.c_str(), ios::out | ios::binary);
		
		if(!output.good())
		{
			cout  << "open file " << file << endl;
			return;
		}
		
		pcl::PointCloud<pcl::PointXYZI> cloud;
		pcl::fromROSMsg (*points_msg, cloud);
		
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
		/*   save points end        */
		
		
		/* save image start    */
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
		cv::imshow("image", cv_ptr->image);
		cv::waitKey(1);
		
		static std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 100};
		imwrite(image_path_str_ + "/data/" + seq_str + ".jpg", cv_ptr->image ,params);
		of_image_stamp_ << timeConvert(image_msg->header.stamp.toSec()) << endl;
		/*   save image end */
	}
	
	//2019-08-23 12:11:39.356720209
	string timeConvert(const double& timeStamp)
	{
		char buf[50]="\0";
		time_t second = timeStamp;
		struct tm *info = localtime(&second);
		strftime(buf, 50, "%Y-%m-%d %H:%M:%S", info);
	
		return string(buf) + "." + to_string(int((timeStamp - second) *1000000000));
	}
	
	string getDate()
	{
		char buf[12]="\0";
		time_t rawtime;
		time(&rawtime);
		struct tm *info = localtime(&rawtime);
		strftime(buf, 12, "%Y_%m_%d", info);
		return string(buf);
	}
	
	string createDataDir(const string& parent)
	{
		string date = getDate();
		size_t seq = 1;
		for(;;)
		{
			stringstream ss;
			ss << parent << "/" << date << "_drive_" << setw(4) << setfill('0') << seq << "_sync";
			fs::path dir(ss.str());
			if(fs::exists(dir))
				++seq;
			else
			{
				fs::create_directories(dir);
				return ss.str();
			}
		}
	}


private:
	string path_str_;
	fs::path fs_path_;
	
	//point cloud
	std::string pointclouds_topic_;
	std::unique_ptr<SubPointCloud2> sub_point_cloud_;
	string points_path_str_;
	string points_stamp_file_name_;
	string points_satrt_stamp_file_name_;
	string points_end_stamp_file_name_;
	ofstream of_points_stamp_,
			 of_points_stamp_start_,
			 of_points_stamp_end;
	//image
	std::string image_topic_;
	std::unique_ptr<SubImage> sub_image_;
	string image_stamp_file_name_;
	string image_path_str_;
	ofstream of_image_stamp_;
	
	std::unique_ptr<Suychronizer> sync_;
};


int main(int argc, char** argv) 
{
	ros::init (argc, argv, "ros2kitti_node");
	Tokitti to_kitti;
	if(to_kitti.init())
		ros::spin();

	return 0;
}
