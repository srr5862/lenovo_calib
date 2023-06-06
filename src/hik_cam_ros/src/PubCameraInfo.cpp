
#include <ros/ros.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/CameraInfo.h>
#include <math.h>

using namespace std;
using namespace cv;

sensor_msgs::CameraInfo getCameraInfo(void){        // extract cameraInfo.
    sensor_msgs::CameraInfo cam;

    vector<double> D{-0.1779536910815181,0.11230081170699428,-0.000414365689274243,9.025272569088937e-05,-0.04036085011312261};

    boost::array<double, 9> K = {
        907.2279243644371, 0.000000, 599.2463987954802,
        0.000000, 907.5567393416758, 521.7126272813557,
        0.000000, 0.000000, 1.000000  
    };
    
    boost::array<double, 12> P = {
        844.5908203125, 0.000000, 597.4036097370554, 0.000000,
        0.000000, 860.7564697265625, 521.6646518703637, 0.000000,
        0.000000, 0.000000, 1.000000, 0.000000
    };
    boost::array<double, 9> r = {1, 0, 0, 0, 1, 0, 0, 0, 1};

    cam.height = 1024;
    cam.width = 1224;
    cam.distortion_model = "plumb_bob";
    cam.D = D;
    cam.K = K;
    cam.P = P;
    cam.R = r;
    cam.binning_x = 0;
    cam.binning_y = 0;
    cam.header.frame_id = "camera";  //frame_id为camera，也就是相机名字
    cam.header.stamp = ros::Time::now();
    cam.header.stamp.nsec = 0;
    return cam;
}

int main(int argc, char** argv) {

  ros::init(argc, argv, "camera_info");  //初始化了一个节点，名字为camera_info
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<sensor_msgs::CameraInfo>("/camera/camera_info", 100);  
  sensor_msgs::CameraInfo camera_info_dyn;
  ros::Rate rate(1);  //点云更新频率0.5Hz

  while (ros::ok())
  {
      camera_info_dyn = getCameraInfo();
      pub.publish(camera_info_dyn); //发布出去
      rate.sleep();
  }
    ros::spin();
    return 0;
}
