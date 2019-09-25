#include <geodesy/utm.h>
#include <geographic_msgs/GeoPointStamped.h>
#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <fstream>
#include<boost/filesystem.hpp>

namespace fs = boost::filesystem;
using namespace std;


int main()
{
	fs::directory_iterator begin("/home/wendao/projects/tools_ws/src/ll2utm/data");
	fs::directory_iterator end;
	for( auto iter = begin; iter != end; ++iter)
	{
		if(fs::extension(*iter) != ".txt")
			continue;
		string inFileName = fs::system_complete(*iter).string();
		ifstream in_file(inFileName);
		if(!in_file.is_open())
		{
			ROS_ERROR("open %s failed!",inFileName.c_str());
			continue;
		}
		
		double latitude, longitude;
		
		string outFileName = inFileName.substr(0,inFileName.find_last_of(".")) + "_utm.txt";
		//cout << outFileName << endl;
		ofstream out_file(outFileName);
		
		out_file << fixed << setprecision(3);
		
		geographic_msgs::GeoPoint ll;
		geodesy::UTMPoint utm;
		while(!in_file.eof())
		{
			for(int i=0; i<3; ++i)
			{
				in_file >> latitude >> longitude ;
				ll.latitude = latitude;
				ll.longitude = longitude;
				ll.altitude = 7.94;
				geodesy::fromMsg(ll, utm);
				out_file << utm.easting << "\t" << utm.northing << "\t\t";
			}
			out_file << "\r\n";
		}
	}
	return 0;
}


