#include <ros/ros.h>
#include<vector>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <cstdio>
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
    string window_name = "Lucas–Kanade Optical Flow Tracking";
    vector<Point2f> points[2];  // point0為特徵點的原来位置，point1為特徵點的新位置
    vector<Point2f> initial;    // 初始化跟蹤點的位置
    vector<Point2f> features;   // 檢測的特徵
    /****************************************/
    // Mat points[2];  // point0為特徵點的原来位置，point1為特徵點的新位置
    // Mat initial;    // 初始化跟蹤點的位置
    // Mat features;   // 檢測的特徵
    /****************************************/
    int maxCount = 500; // 檢測的最大角點數目
    double qLevel = 0.01;   // 特徵檢測的等級（一般於0.01-0.1之間）
    double minDist = 10.0;  // 兩特徵點之間的最小距離，小於此距離的點要被忽略
    vector<uchar> status;   // 跟蹤特徵的狀態，特徵的流發現為1，否則為0
    vector<float> err;
    double deltaDist;
    int count = 0;

public:
    LK_OpticalFlow(ros::NodeHandle);
    void Tracking(Mat &, Mat &);
    bool AddNewPoints();
    bool AcceptTrackedPoint(int);
};

//輸出相應信息和OpenCV版本-----
//static void helpinformation()
//{
//    cout <<"\n\n\t\t\t 光流法跟蹤運動目標檢測\n"
//         <<"\n\n\t\t\t 當前使用的OpenCV版本為：" << CV_VERSION 
//         <<"\n\n" ;
//}

//main( )函數，程序入口
LK_OpticalFlow::LK_OpticalFlow(ros::NodeHandle nh)
{
	// rs2::pipeline pipe;
	// rs2::config cfg;
	const int w = 640;
	const int h = 480;
	// cfg.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, 30);
	// pipe.start(cfg);
    //加載使用的影片文件，放在項目程序運行文件下
    cv::VideoCapture capture(0);
    //顯示信息函數
    // helpinformation();
    // Camera讀取文件開關
    if(capture.isOpened())  
    {
        while(true)
        {
			// rs2::frameset frames = pipe.wait_for_frames();
			// rs2::frame color_frame = frames.get_color_frame();
			// Mat color_image(Size(w, h), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
			// frame = color_image;
            capture >> frame;

            if(!frame.empty())
            { 
                Tracking(frame, result);
                // count = count + 1;
                // cout << count << endl;
            }
            else
            { 
                printf(" --(!) No captured frame -- Break!");
                break;
            }
            int c = waitKey(50);
            if( (char)c == 27 )
            {
                break; 
            } 
        }
    }
    // return 0;
}

// parameter: frame 輸入的影片帧
//            output 有跟蹤結果的影片帧
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
    calcOpticalFlowPyrLK(gray_prev, gray, points[0], points[1], status, err);

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
    }

    // 把當前跟蹤結果作為下一此參考
    swap(points[1], points[0]);
    swap(gray_prev, gray);  
    imshow(window_name, output);
}

//  檢測新點是否應該被添加
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
    ros::init(argc, argv, "LK_OpticalFlow");
    ros::NodeHandle nh;
    LK_OpticalFlow LK_OpticalFlow(nh);
    ros::spin();
}