#include <ros/ros.h>
#include <cstdio>
#include <iostream>
#include <fstream>
#include <std_msgs/Float64MultiArray.h>
using namespace ros;
using namespace std;

int i = 0;
std::ofstream myfile;

void point_data_csv(const std_msgs::Float64MultiArray::ConstPtr& point_msg){
	//std::ofstream myfile;
	myfile.open ("/home/graduationv2/DynamicTracking-LKOpticalFlow/src/LKOpticalFlow/csv/point_data.txt", std::ios_base::app);
	//myfile << ",origin_point.x,origin_point.y,origin_point.z,target_point.x,target_point.y,target_point.z\n";
	//myfile << "Data no." << i << ",";
	for(int k = 0; k < point_msg->data.size(); k++){
		if(!isnan(point_msg->data[k])){
			myfile << point_msg->data[k] << ",";
		}
	}
	i++;
	myfile << "\n";
    myfile.close();
}

int main( int argc, char* argv[] ){
	ros::init(argc, argv, "point_data_csv");
    ros::NodeHandle nh;
	ros::Subscriber point_sub = nh.subscribe("lk/point_data", 1000, point_data_csv);	

    ros::spin();
	return 0;
}
