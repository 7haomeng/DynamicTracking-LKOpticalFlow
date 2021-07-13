#include <ros/ros.h>
// #include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

#include <iostream>
#include <cstdio>
#include <math.h>
// #include <vector>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/pcl_macros.h>
#include <pcl/common/common.h>

// #include <sensor_msgs/image_encodings.h>
using namespace ros;
using namespace std;
using namespace cv;
// using namespace pcl;

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudXYZRGBPtr;

typedef pcl::PointXYZRGB originpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> OriginPointCloudXYZRGBtoROSMsg;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr OriginPointCloudXYZRGBtoROSMsgPtr;

typedef pcl::PointXYZRGB targetpoint;
typedef pcl::PointCloud<pcl::PointXYZRGB> TargetPointCloudXYZRGBtoROSMsg;
typedef pcl::PointCloud<pcl::PointXYZRGB>::Ptr TargetPointCloudXYZRGBtoROSMsgPtr;

class LK_OpticalFlow
{
private:
    ros::NodeHandle my_nh;

    cv::Mat frame;
    Mat result;
    Mat gray;   // 當前圖片
    Mat gray_prev;  // 預測圖片
    cv_bridge::CvImagePtr cv_ptr; 
    string window_name = "Lucas–Kanade Optical Flow Tracking";
    vector<Point2f> points[2];  // point0為特徵點的原来位置，point1為特徵點的新位置
    vector<Point2f> initial;    // 初始化跟蹤點的位置
    vector<Point2f> features;   // 檢測的特徵
    int maxCount = 500; // 檢測的最大角點數目
    double qLevel = 0.01;   // 特徵檢測的等級（一般於0.01-0.1之間）
    double minDist = 10.0;  // 兩特徵點之間的最小距離，小於此距離的點要被忽略
    vector<uchar> status;   // 跟蹤特徵的狀態，特徵的流發現為1，否則為0
    vector<float> err;

    clock_t t1, t2, delta_time;
    int count = 0;
    double deltaDist;

	float x_op_original;
	float y_op_original;
	float z_op_original;
	float x_op_target;
	float y_op_target;
	float z_op_target;
	float distance;

    geometry_msgs::Vector3 feature_points_msg;
    sensor_msgs::PointCloud2 originPointCloudtoROSMsg;
    sensor_msgs::PointCloud2 targetPointCloudtoROSMsg;
	sensor_msgs::ImagePtr output_image;
	std_msgs::Float64MultiArray origin_point;
	std_msgs::Float64MultiArray target_point;
	std_msgs::Float64MultiArray point_data;
    
    PointCloudXYZRGBPtr cloud_ptr;
    originpoint origin_pointcloud;
    OriginPointCloudXYZRGBtoROSMsgPtr origincloudptr_to_ROSMsg;
    targetpoint target_pointcloud;
    TargetPointCloudXYZRGBtoROSMsgPtr targetcloudptr_to_ROSMsg;

public:
    LK_OpticalFlow(ros::NodeHandle);
    void image_raw_Callback(const sensor_msgs::ImageConstPtr&);
    void Tracking(Mat &, Mat &);
    bool AddNewPoints();
    bool AcceptTrackedPoint(int);
	float ComputeDistance(float, float, float, float, float, float);
	ros::Subscriber image_raw_sub;
    ros::Publisher feature_points_pub;
    ros::Publisher origin_pointcloud_pub;
    ros::Publisher target_pointcloud_pub;
	ros::Publisher origin_point_pub;	
	ros::Publisher target_point_pub;
	ros::Publisher point_data_pub;
	image_transport::Publisher output_image_pub;
    // ros::Publisher test_image_pub;
};

LK_OpticalFlow::LK_OpticalFlow(ros::NodeHandle nh)
{
    image_raw_sub = nh.subscribe("/camera/color/image_raw", 1, &LK_OpticalFlow::image_raw_Callback,this);
    // image_raw_sub = nh.subscribe("/camera/color/image_raw", 1, &LK_OpticalFlow::image_raw_Callback,this);
    feature_points_pub = nh.advertise<geometry_msgs::Vector3>("lk/feature_points", 1);
    // feature_points_pub = nh.advertise<geometry_msgs::Point32>("feature_points", 10);
    origin_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lk/origin_pointcloud", 1);
    target_pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lk/target_pointcloud", 1);
	origin_point_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/origin_point", 1);
	target_point_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/target_point", 1);
	point_data_pub = nh.advertise<std_msgs::Float64MultiArray>("lk/point_data", 1);

	image_transport::ImageTransport it(nh);
	output_image_pub = it.advertise("lk/camera/LK_OpticalFlow_image", 1);

    //cv::namedWindow(window_name);
	const int width = 640; // 設定影像尺寸(寬w，高h)
	const int high = 480;

    my_nh = nh;
    cloud_ptr = PointCloudXYZRGBPtr(new PointCloudXYZRGB);
    origincloudptr_to_ROSMsg = OriginPointCloudXYZRGBtoROSMsgPtr(new OriginPointCloudXYZRGBtoROSMsg);
    targetcloudptr_to_ROSMsg = TargetPointCloudXYZRGBtoROSMsgPtr(new TargetPointCloudXYZRGBtoROSMsg);    
}

void LK_OpticalFlow::image_raw_Callback(const sensor_msgs::ImageConstPtr& image_msg)
{

    t1 = clock()*0.001;

    cv_ptr = cv_bridge::toCvCopy(image_msg, "bgr8");
    frame = cv_bridge::toCvCopy(image_msg, "bgr8")->image;

    // TODO: wait for pointcloud2 topic
    boost::shared_ptr<sensor_msgs::PointCloud2 const> tmp_ptr;
    sensor_msgs::PointCloud2 cloud_msg;
    tmp_ptr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>("/camera/depth_registered/points", my_nh);
    if(tmp_ptr != NULL){
        cloud_msg = *tmp_ptr;
    }

    // Convert ROS pointcloud2 topic to PCL pointcloud
    // include pcl_conversion.h --> pcl::fromROSMsg(XXXXXXX)
    pcl::fromROSMsg(cloud_msg, *cloud_ptr);
    

    // test_image_pub.publish(cv_ptr->toImageMsg());

    if(!frame.empty())
    {
        Tracking(frame, result);
        t2 = clock()*0.001;
        delta_time = t2-t1;
        //cout << "ClockDeltaTime : "<< delta_time << "ms" << endl;
		//cout << "FPS : "<< 1 / delta_time * 1000 << endl;
        count=count+1;           
    }
    else
    { 
        printf("No captured frame -- Break!\n");
        return;
    }
    int c = waitKey(50);
    if( (char)c == 27 )
    {
        return; 
    } 
}


void LK_OpticalFlow::Tracking(Mat &frame, Mat &output)
{
    cvtColor(frame, gray, CV_BGR2GRAY);
    frame.copyTo(output);
    // 添加特徵點
    if (AddNewPoints()) 
    {
        //角點檢測
            //image:輸入圖像(gray)
            //corners:輸出角點vector(features)
            //maxCorners:檢測的最大角點數目(maxCount)
            //qualityLevel:特徵檢測的等級,一般於0.01-0.1之間(qLevel)
            //minDistance:兩特徵點之間的最小距離，小於此距離的點要被忽略(minDist)
        goodFeaturesToTrack(gray, features, maxCount, qLevel, minDist);
        
        // cout << "============features vector============" << endl;
        // cout << "features : " << features << endl;
        // cout << "====================================\n" << endl;

        points[0].insert(points[0].end(), features.begin(), features.end());
        // cout << "============points[0] vector============" << endl;
        // cout << "points[0] : " << points[0] << endl;
        // cout << "====================================\n" << endl;

        initial.insert(initial.end(), features.begin(), features.end());
        // cout << "============initial vector============" << endl;
        // cout << "initial : " << initial << endl;
        // cout << "====================================\n" << endl;
    }

    if (gray_prev.empty())
    {
        // cout << gray_prev <<endl;
        gray.copyTo(gray_prev);
    }
    // LK-光流法運動估計
        //prevImg:計算光流的前一偵圖片(gray_prev）
        //nextImg:下一偵圖片(gray)
        //prevPts:前一偵圖片中的特徵點（角點）,
        //nextPts:輸出特徵點於下一偵圖片的新位置
        //status:若兩偵之間的特徵點有發生變化（有光流法現象）則為1，否則為0
        //err:兩偵之間特徵點位置的誤差
    calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err); // LK-光流法運動估計
    // cout << "w: " << gray.cols << ", h: " << gray.rows << endl;
    // 去掉一些不好的特徵點
    int k = 0;
    for (size_t i=0; i<points[1].size(); i++)
    {
        // cout << "points[1][" << i <<"]:" << points[1][i] << endl;
        if (AcceptTrackedPoint(i))
        {
            // printf("%hhu\n",status[i]);
            // cout << deltaDist <<endl;
            initial[k] = initial[i];
            points[1][k++] = points[1][i];
        }
        // cout << "k_points[1][" << k++ << "]:" << points[1][k++] << endl;
        // cout <<endl;
    }
    points[1].resize(k);
    initial.resize(k);
    // 顯示特徵點和運動軌跡
    for (size_t i=0; i<points[1].size(); i++)
    {
        line(output, initial[i], points[1][i], Scalar(255, 0, 0));
        circle(output, initial[i], 3, Scalar(0, 255, 0), -1);
        circle(output, points[1][i], 3, Scalar(0, 0, 255), -1);
        // cout<< "==================================" << endl;
        // cout << "InitialPoint : " << initial[i] << endl;
        // cout << "TerminalPoint : " << points[1][i] << endl;
        // cout<< "==================================\n" << endl;
        // double opticaldistence = sqrt(pow((points[0][i].x - points[1][i].x),2)+pow((points[0][i].y - points[1][i].y),2));
        // cout<< "**************************" << endl;
        // cout << "opticaldistence : " << opticaldistence << endl;
        
        // ROS_INFO("Origin feature point: x=%f, y=%f", (float)initial[i].x, (float)initial[i].y);
        // ROS_INFO("Target feature point: x=%f, y=%f\n", (float)points[1][i].x, (float)points[1][i].y);
        // feature_points_msg.x = points[1][i].x;
        // feature_points_msg.y = points[1][i].y;
        // feature_points_msg.z = 0;
        // feature_points_pub.publish(feature_points_msg);

        int initial_index = 640 * initial[i].y + initial[i].x; //計算Origin point單一pixel-wise於640x480第幾個
        if(pcl_isfinite(cloud_ptr->points[initial_index].x)) 
        {   //ROS_INFO("\033[1;32m\nOrigin point[x y z] : [%lf %lf %lf]\033[0m",cloud_ptr->points[initial_index].x,cloud_ptr->points[initial_index].y,cloud_ptr->points[initial_index].z);

            origin_pointcloud.r = 0;
            origin_pointcloud.g = 255;
            origin_pointcloud.b = 0;
            origin_pointcloud.x = cloud_ptr->points[initial_index].x;
            origin_pointcloud.y = cloud_ptr->points[initial_index].y;
            origin_pointcloud.z = cloud_ptr->points[initial_index].z;
            origincloudptr_to_ROSMsg->points.push_back(origin_pointcloud);

			//printf("origin_pointcloud.x : %d\n", cloud_ptr->points[initial_index].x);

			//origin_point.data.push_back(cloud_ptr->points[initial_index].x);
			//origin_point.data.push_back(cloud_ptr->points[initial_index].y);
			//origin_point.data.push_back(cloud_ptr->points[initial_index].z);
			//origin_point_pub.publish(origin_point);
			//origin_point.data.clear();

        }
		

        int target_index = 640 * points[1][i].y + points[1][i].x; //計算Target point單一pixel-wise於640x480第幾個
        if(pcl_isfinite(cloud_ptr->points[target_index].x))
        {
			//ROS_INFO("\033[1;31m\nTarget point[x y z] : [%lf %lf %lf]\n\033[0m",cloud_ptr->points[target_index].x,cloud_ptr->points[target_index].y,cloud_ptr->points[target_index].z);
            //cout << "Target point:\nx: " << cloud_ptr->points[target_index].x;
            //cout << "\ny: " << cloud_ptr->points[target_index].y;
            //cout << "\nz: " << cloud_ptr->points[target_index].z << endl;
            //cout << "**************" << endl;
            //cout << endl;
            
            // target_pointcloud.r = cloud_ptr->points[target_index].r;
            // target_pointcloud.g = cloud_ptr->points[target_index].g;
            // target_pointcloud.b = cloud_ptr->points[target_index].b;
            target_pointcloud.r = 255;
            target_pointcloud.g = 0;
            target_pointcloud.b = 0;
            target_pointcloud.x = cloud_ptr->points[target_index].x;
            target_pointcloud.y = cloud_ptr->points[target_index].y;
            target_pointcloud.z = cloud_ptr->points[target_index].z;
            targetcloudptr_to_ROSMsg->points.push_back(target_pointcloud);

			//target_point.data.push_back(cloud_ptr->points[target_index].x);
			//target_point.data.push_back(cloud_ptr->points[target_index].y);
			//target_point.data.push_back(cloud_ptr->points[target_index].z);
			//target_point_pub.publish(target_point);
			//target_point.data.clear();

        }
		
		if((pcl_isfinite(cloud_ptr->points[initial_index].x)) && (pcl_isfinite(cloud_ptr->points[target_index].x)))
        {
			x_op_original = cloud_ptr->points[initial_index].x;
			y_op_original = cloud_ptr->points[initial_index].y;
			z_op_original = cloud_ptr->points[initial_index].z;
			x_op_target = cloud_ptr->points[target_index].x;
			y_op_target = cloud_ptr->points[target_index].y;
			z_op_target = cloud_ptr->points[target_index].z;
			ROS_INFO("\033[1;32mOptical Flow points : [ %lf %lf %lf %lf %lf %lf ]\033[0m",x_op_original,y_op_original,z_op_original,x_op_target,y_op_target,z_op_target);
			point_data.data.push_back(cloud_ptr->points[initial_index].x);
			point_data.data.push_back(cloud_ptr->points[initial_index].y);
			point_data.data.push_back(cloud_ptr->points[initial_index].z);
			point_data.data.push_back(cloud_ptr->points[target_index].x);
			point_data.data.push_back(cloud_ptr->points[target_index].y);
			point_data.data.push_back(cloud_ptr->points[target_index].z);
			point_data_pub.publish(point_data);
			point_data.data.clear();
			
			ROS_INFO("\033[1;31mDistance : [ %lf ]\n\033[0m",ComputeDistance(x_op_original,y_op_original,z_op_original,x_op_target,y_op_target,z_op_target));
			//ComputeDistance(x_op_original,y_op_original,z_op_original,x_op_target,y_op_target,z_op_target);
		}
    }
    pcl::toROSMsg(*(origincloudptr_to_ROSMsg), originPointCloudtoROSMsg);
    originPointCloudtoROSMsg.header.frame_id = "map";
    originPointCloudtoROSMsg.header.stamp = ros::Time::now();
    origin_pointcloud_pub.publish(originPointCloudtoROSMsg);
    origincloudptr_to_ROSMsg->points.clear();

    pcl::toROSMsg(*(targetcloudptr_to_ROSMsg), targetPointCloudtoROSMsg);
    targetPointCloudtoROSMsg.header.frame_id = "map";
    targetPointCloudtoROSMsg.header.stamp = ros::Time::now();
    target_pointcloud_pub.publish(targetPointCloudtoROSMsg);
    targetcloudptr_to_ROSMsg->points.clear(); // ros::Duration(0.033).sleep();
    // 把當前跟蹤結果作為下一此參考
    swap(points[1], points[0]);
    swap(gray_prev, gray);
    //cv::imshow("Lucas–Kanade Optical Flow Tracking", output);
	output_image = cv_bridge::CvImage(std_msgs::Header(), "bgr8", output).toImageMsg();
	output_image_pub.publish(output_image);
}


//决定哪些跟蹤點被接受
// return: 是否被添加標誌
bool LK_OpticalFlow::AddNewPoints()
{
    return points[0].size() <= 10;
}

//决定哪些跟蹤點被接受
bool LK_OpticalFlow::AcceptTrackedPoint(int i)
{
    deltaDist = abs(points[0][i].x - points[1][i].x) + abs(points[0][i].y - points[1][i].y);
    return status[i] && deltaDist > 2;
}

float LK_OpticalFlow::ComputeDistance(float x_o, float y_o, float z_o, float x_t, float y_t, float z_t)
{
	distance = sqrt(pow((x_t - x_o),2) + pow((y_t - y_o),2) + pow((z_t - z_o),2));
	return distance;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LKOpticalFlow_Realsense");
    ros::NodeHandle nh;
    LK_OpticalFlow LK_OpticalFlow(nh);
    ros::spin();
}
