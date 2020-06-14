#include <iostream>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/impl/plane_clipper3D.hpp>
#include <pcl/registration/registration.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/transforms.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>

using PointT = pcl::PointXYZI;

pcl::PointCloud<PointT>::Ptr plane_clip(const pcl::PointCloud<PointT>::Ptr& src_cloud, const Eigen::Vector4f& plane, bool negative)
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

pcl::PointCloud<PointT>::Ptr radius_filter(const pcl::PointCloud<PointT>::Ptr in, float max_radius)
{
	pcl::ExtractIndices<PointT> extracter;
	extracter.setInputCloud(in);
	pcl::PointIndices indices;
	indices.indices.reserve(in->points.size());
	#pragma omp for
	for (size_t i = 0; i < in->points.size(); i++)
	{
		float radius2 = in->points[i].x*in->points[i].x + in->points[i].y*in->points[i].y;
		if(radius2 > max_radius*max_radius)
			indices.indices.push_back(i); 
	}
    pcl::PointCloud<PointT>::Ptr out(new pcl::PointCloud<PointT>);
	extracter.setIndices(boost::make_shared<pcl::PointIndices>(indices));
	extracter.setNegative(true); //ture to remove the indices
	extracter.filter(*out);
    return out;
}


/*
boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<PointT, PointT>> gicp(new pcl::GeneralizedIterativeClosestPoint<PointT, PointT>());
boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
ndt->setTransformationEpsilon(0.01);
ndt->setResolution(ndt_resolution);

registration->setInputTarget(keyframe);
registration->setInputSource(filtered);
pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
registration->align(*aligned, prev_trans); //output the registrated pointcloud
if(!registration->hasConverged()) {
    Eigen::Matrix4f trans = registration->getFinalTransformation();
*/

bool next_iteration = false;
void keyboardEvent(const pcl::visualization::KeyboardEvent &event,void *viewer_void)
{
    //pcl::visualization::PCLVisualizer *viewer = (pcl::visualization::PCLVisualizer *)viewer_void;
    
    //std::cout << "keybord event: " << event.getKeySym() << std::endl;
    if(event.getKeySym() == "space" && event.keyDown())
        next_iteration = true;
}

void mouseEventOccurred (const pcl::visualization::MouseEvent &event, void* viewer_void)
{
    if (event.getButton() == pcl::visualization::MouseEvent::LeftButton && 
        event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
    {
        //std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
        //next_iteration = true;
    }
}

float deg2rad(float deg)
{
    return deg/180.0*M_PI;
}

int main(int argc, char** argv) 
{
    // *****Initialize ROS
    ros::init (argc, argv, "register_demo");
    ros::NodeHandle nh, nh_private("~");
    
    if(argc <2)
    {
    	ROS_ERROR("please input pcd file path...");
    	return 0;
    }
 
    //*****load two pcd files
    pcl::PointCloud<PointT>::Ptr cloud_raw (new pcl::PointCloud<PointT>);//创建点云指针，存储点坐标xyz
    if (pcl::io::loadPCDFile<PointT> (argv[1], *cloud_raw) == -1)
    {
        PCL_ERROR ("Couldn't read file %s ^.^\n",argv[1]);
        return (-1);
    }

    //std::cout << "raw pointcloud size : " << cloud_raw->width*cloud_raw->height<< std::endl;
    //cloud_raw = radius_filter(cloud_raw,50);
    //std::cout << "filtered pointcloud size : " << cloud_raw->width*cloud_raw->height<< std::endl;

    //自定义变换
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(5,0,0)); 
    tf::Quaternion q;
    q.setRPY(deg2rad(20.0),deg2rad(20.0),deg2rad(0.0));
    transform.setRotation(q);

    //  tf::Transform 转 Eigen::Matrix4f
    Eigen::Translation3f tl_btol(transform.getOrigin().getX(), transform.getOrigin().getY(), transform.getOrigin().getZ());
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getEulerYPR(yaw, pitch, roll);
    Eigen::AngleAxisf rot_x_btol(roll, Eigen::Vector3f::UnitX());
    Eigen::AngleAxisf rot_y_btol(pitch, Eigen::Vector3f::UnitY());
    Eigen::AngleAxisf rot_z_btol(yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix4f true_trans = (tl_btol * rot_z_btol * rot_y_btol * rot_x_btol).matrix();

    //由原始点云变换得到新点云
    pcl::PointCloud<PointT>::Ptr cloud_new (new pcl::PointCloud<PointT>);

    //cloud_raw经转换矩阵transform得到cloud_new，  cloud_new = transform*cloud_raw；左乘
    pcl_ros::transformPointCloud(*cloud_raw, *cloud_new, transform);

    //std::cout << "target pointcloud size : " << cloud_raw->width*cloud_raw->height<< std::endl;
    //std::cout << "source pointcloud size : " << cloud_new->width*cloud_new->height<< std::endl;
    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    //viewer->setCameraPosition(0, 0, -2, 0, -1, 0, 0);
    //viewer->initCameraParameters (); //导致不显示
    viewer->registerKeyboardCallback(&keyboardEvent,(void*)viewer.get()); //捕获键盘事件
    viewer->registerMouseCallback(&mouseEventOccurred, (void*)viewer.get()); //捕获鼠标事件

    viewer->setBackgroundColor (255, 255, 255);
    viewer->addCoordinateSystem (3.0);
	
    viewer->addPointCloud<PointT> (cloud_raw, "source");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255, "source");

    viewer->addPointCloud<PointT> (cloud_new, "target");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "target");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,255,0,0, "target");

    //NDT
    boost::shared_ptr<pcl::NormalDistributionsTransform<PointT, PointT>> ndt(new pcl::NormalDistributionsTransform<PointT, PointT>());
    ndt->setTransformationEpsilon(0.01);
    ndt->setResolution(0.1);
    ndt->setStepSize (0.1);

    //ICP
    boost::shared_ptr<pcl::IterativeClosestPoint<PointT, PointT>> icp(new pcl::IterativeClosestPoint<PointT, PointT>());
	icp->setMaxCorrespondenceDistance(100.0);
	icp->setTransformationEpsilon(1e-10);
	icp->setEuclideanFitnessEpsilon(0.005);

    boost::shared_ptr<pcl::Registration<PointT, PointT>> registration = icp;
    registration->setInputSource(cloud_raw);
    registration->setInputTarget(cloud_new);

    Eigen::Matrix4f trans = Eigen::Matrix4f::Identity(); 
    
    while(!viewer->wasStopped())
    {
        while(!next_iteration)
        {
            viewer->spinOnce(100);
            usleep(100000);;
        }

        next_iteration = false;
        
        registration->setMaximumIterations(1);

        //对齐的结果为source向target的变换
        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());

        registration->align(*aligned); //output the registrated pointcloud

        //trans为target相对与souce的估计变换矩阵 trans*source ～= target
        //pcl::transformPointCloud(*source, *target～, trans); 其中target～ 与aligned相同
        trans = registration->getFinalTransformation();

        std::cout << "registeration score:" << registration->getFitnessScore() 
                  << "\tisConverged:" <<  registration->hasConverged() << std::endl;

        registration->setInputSource(aligned);

        viewer->removePointCloud("source");
        viewer->addPointCloud<PointT> (aligned, "source");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255, "source");  
    }

    //
    viewer->spin();
    return 0;
}
/* 手动等步长旋转
 while(!viewer->wasStopped())
{
    static int i = 1;
    while(!next_iteration)
    {
        viewer->spinOnce(100);
        usleep(100000);;
    }

    next_iteration = false;

    float x = i*1.0/10*5;
    float roll = i*1.0/10*20.0;
    float pitch = i*1.0/10*20.0;
    i++;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(x,0,0)); 
    tf::Quaternion q;
    q.setRPY(deg2rad(roll),deg2rad(pitch),deg2rad(0.0));
    transform.setRotation(q);

    pcl::PointCloud<PointT>::Ptr aligned (new pcl::PointCloud<PointT>);
    pcl_ros::transformPointCloud(*cloud_raw, *aligned, transform);

    viewer->removePointCloud("source");
    viewer->addPointCloud<PointT> (aligned, "source");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "source");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR,0,0,255, "source");  
}
*/