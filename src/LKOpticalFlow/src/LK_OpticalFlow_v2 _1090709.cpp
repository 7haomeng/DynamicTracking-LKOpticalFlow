#include <ros/ros.h>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstdio>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "geometry_msgs/Vector3.h"
#include "sensor_msgs/Image.h"
using namespace ros;
using namespace std;
using namespace cv;

class LK_OpticalFlow
{
private:
    rs2::pipeline pipe; // 建構一個RealSense抽象設備的管道以容納擷取到的影像
	rs2::config cfg; // 創建自定義參數以配置管道
    Mat frame;
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
    geometry_msgs::Vector3 feature_points_msg;

public:
    LK_OpticalFlow(ros::NodeHandle);
    void image_raw_Callback(const sensor_msgs::ImageConstPtr&);
    void Tracking(Mat &, Mat &);
    bool AddNewPoints();
    bool AcceptTrackedPoint(int);
    ros::Publisher feature_points_pub;
    // ros::Subscriber image_raw_sub;
};
// VideoWriter writer_opticalflow("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, Size(1920, 1080));
// VideoWriter writer_opticalflow("VideoTest.avi", CV_FOURCC('D','I','V','X'), 30.0, Size(1920, 1080));

LK_OpticalFlow::LK_OpticalFlow(ros::NodeHandle nh)
{
    
    feature_points_pub = nh.advertise<geometry_msgs::Vector3>("feature_points", 10);

	const int width = 640; // 設定影像尺寸(寬w，高h)
	const int high = 480;
    // cv::namedWindow(window_name);
    // const int width = 640; // 設定影像尺寸(寬w，高h)
	// const int high = 480;
	cfg.enable_stream(RS2_STREAM_COLOR, width, high, RS2_FORMAT_BGR8, 30); // BGR888格式彩色影像 30fps 6/1
	pipe.start(cfg); // 根據設定值啟動指定串流影像 6/1
    // VideoWriter writer_opticalflow("VideoTest.avi", CV_FOURCC('M', 'J', 'P', 'G'), 25.0, Size(1920, 1080));
    
    // image_raw_sub = nh.subscribe("/camera/color/image_raw", 1000, &LK_OpticalFlow::image_raw_Callback,this);
    // while(true)
    while(ros::ok())
    {
        t1 = clock()*0.001;
        // int delta_time_temp = 0;
        // clock_t delta_time_temp = 0;
        
        // double BeginTime = double(ros::Time::now().toNSec())*0.000001;
        // cout << "BeginTime : "<< BeginTime << endl;
        
		rs2::frameset frames = pipe.wait_for_frames(); // 等待下一組影像 6/1
		rs2::frame color_frame = frames.get_color_frame(); // 取得每一張彩色影像 6/1
		Mat color_image(Size(width, high), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP); //建立OpenCV Mat格式之彩色影像
		frame = color_image;
        // writer << frame;
        // printf(doublelsense D435 successfully!");
        // double BeginTime = double(ros::Time::now().toNSec())*0.000001;
        // cout << "BeginTime : "<< BeginTime << endl;
        
        if(!frame.empty())
        {
            Tracking(frame, result);
            t2 = clock()*0.001;
            // double EndTime = double(ros::Time::now().toNSec())*0.000001;
            // cout << "EndTime : "<< EndTime << endl;
            // ros::Time EndTime = ros::Time::now();
            // cout << "EndTime : "<< EndTime << endl;
            
            // double DeltaTime = EndTime - BeginTime;
            // cout << "DeltaTime : "<< DeltaTime << endl;
            delta_time = t2-t1;
            // cout << "ClockDeltaTime : "<< delta_time << "ms" << endl;

            // cout << "delta_time : "<< delta_time << endl;
            // for(count = 0; count<100; count++)
            // {
                // delta_time_temp = delta_time;
            // cout << "delta_time_temp : "<< delta_time_temp << endl;

                // delta_time_temp = delta_time_temp + delta_time;
            // cout << "delta_time_temp : "<< delta_time_temp << endl;
            // }
            // clock_t avg_time = delta_time_temp/count;
            // cout << "avg_time : " << avg_time << endl;
            count=count+1;
            // cout<< "count :"<< count << endl;
            
        }
        else
        { 
            printf("No captured frame -- Break!\n");
            break;
        }
        int c = waitKey(50);
        if( (char)c == 27 )
        {
            break; 
        } 
    }
    // return 0;
}

void LK_OpticalFlow::Tracking(Mat &frame, Mat &output)
{
    ROS_INFO("YES");
    cvtColor(frame, gray, CV_BGR2GRAY);
    frame.copyTo(output);
    ROS_INFO("NICE");
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
    ROS_INFO("GOOD");
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
        line(output, initial[i], points[1][i], Scalar(0, 0, 255));
        circle(output, initial[i], 3, Scalar(255, 0, 0), -1);
        circle(output, points[1][i], 3, Scalar(0, 255, 0), -1);
        // cout<< "==================================" << endl;
        // cout << "InitialPoint : " << initial[i] << endl;
        // cout << "TerminalPoint : " << points[1][i] << endl;
        // cout<< "==================================\n" << endl;
        // double opticaldistence = sqrt(pow((points[0][i].x - points[1][i].x),2)+pow((points[0][i].y - points[1][i].y),2));
        // cout<< "**************************" << endl;
        // cout << "opticaldistence : " << opticaldistence << endl;

        ROS_INFO("Org feature point: x=%f, y=%f", (float)initial[i].x, (float)initial[i].y);
        ROS_INFO("End feature point: x=%f, y=%f\n", (float)points[1][i].x, (float)points[1][i].y);
        feature_points_msg.x = points[1][i].x;
        feature_points_msg.y = points[1][i].y;
        feature_points_msg.z = 0;
        feature_points_pub.publish(feature_points_msg);

    }
    ROS_INFO("PERFECT");
    // 把當前跟蹤結果作為下一此參考
    swap(points[1], points[0]);
    swap(gray_prev, gray);
    imshow("window_name", output);
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LKOpticalFlow_Realsense");
    ros::NodeHandle nh;
    LK_OpticalFlow LK_OpticalFlow(nh);
    ros::spin();
}
