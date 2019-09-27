#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/filters/extract_indices.h>

class PcdFilter
{
public:
	using PointT = pcl::PointXYZI;
	PcdFilter(){}
	~PcdFilter(){}
	
	bool init()
	{
		ros::NodeHandle nh,nh_private("~");
		mPublisher = nh.advertise<sensor_msgs::PointCloud2>("/point_cloud",10);
		mPcdFileName = nh_private.param<std::string>("pcd_file","");
		mClipRangeUp = nh_private.param<float>("clip_range_up",1.0);
		mClipRangeDown = nh_private.param<float>("clip_range_down",-1.0);
		
		if(mPcdFileName.empty())
		{
			ROS_ERROR("no pcd file!");
			return false;
		}
		return true;
	}
	
	pcl::PointCloud<PointT>::Ptr planeClip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative)
	{
		pcl::PlaneClipper3D<PointT> clipper(plane);
		pcl::PointIndices::Ptr indices(new pcl::PointIndices);

		clipper.clipPointCloud3D(*src_cloud, indices->indices);

		pcl::PointCloud<PointT>::Ptr dst_cloud(new pcl::PointCloud<PointT>);

		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(src_cloud);
		extract.setIndices(indices);
		extract.setNegative(negative);
		extract.filter(*dst_cloud);
		return dst_cloud;
	}
	
	bool loadPCDFile()
	{
		mRawPointcloudPtr.reset(new pcl::PointCloud<PointT>);
		if (pcl::io::loadPCDFile<PointT> (mPcdFileName, *mRawPointcloudPtr) == -1)
		{
			PCL_ERROR ("Couldn't read file %s ^.^\n",mPcdFileName.c_str());
			return false;
		}
		std::cout << "size of point cloud : " << mRawPointcloudPtr->width * mRawPointcloudPtr->height << std::endl;
		return true;
	}
	
	void filter()
	{
		mfilteredCloudPtr.reset(new pcl::PointCloud<PointT>);
		float sensor_height = 1.8;
		mfilteredCloudPtr = planeClip(mRawPointcloudPtr, Eigen::Vector4f(0.0f, 0.0f, 1.0f, mClipRangeUp), false);
		mfilteredCloudPtr = planeClip(mRawPointcloudPtr, Eigen::Vector4f(0.0f, 0.0f, 1.0f, mClipRangeDown), true);
	}
	
	void showPointcloud()
	{
		mViewer.reset(new pcl::visualization::PCLVisualizer("3D Viewer"));
		mViewer->setBackgroundColor (0, 0, 0); 
		mViewer->addPointCloud<PointT> (mfilteredCloudPtr, "sample cloud");
		mViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
		mViewer->addCoordinateSystem (1.0);
	}
	
	void flushViewer(int ms)
	{
		mViewer->spinOnce(ms);
	}
	
	void publishPointcloud()
	{
		sensor_msgs::PointCloud2 pointcloud_msg;
		pcl::toROSMsg (*mfilteredCloudPtr, pointcloud_msg);
		
		pointcloud_msg.header.stamp = ros::Time::now();
		pointcloud_msg.header.frame_id = "lidar";
		mPublisher.publish(pointcloud_msg);
	}
	

private:
	ros::Publisher mPublisher;
	pcl::PointCloud<PointT>::Ptr mRawPointcloudPtr;
	pcl::PointCloud<PointT>::Ptr mfilteredCloudPtr;
	boost::shared_ptr<pcl::visualization::PCLVisualizer> mViewer;
	std::string mPcdFileName;
	float mClipRangeUp;
	float mClipRangeDown;
};


int main(int argc, char** argv) 
{
	ros::init(argc,argv,"pcdfilter_node");
	
	PcdFilter filter;
	if(!filter.init())
		return 0;
	if(!filter.loadPCDFile())
		return 0;
	filter.filter();
	filter.showPointcloud();
	
	ros::Rate r(10);
	while(ros::ok())
	{
		filter.publishPointcloud();
		filter.flushViewer(100);
		r.sleep();
	}
	return 0;
}










