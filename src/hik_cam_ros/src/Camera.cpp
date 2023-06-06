#include "Camera.hpp"
#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <nodelet/nodelet.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int argc, char *argv[])
{
    cv::Mat src, resize_src;
    ros::init(argc, argv, "hik_cam_capture");
    ros::NodeHandle nh("~");
    
    // params
    int img_width;
    int img_height;
    string img_topic;
    string frame_id;

    nh.getParam("pub_img_width", img_width);
    nh.getParam("pub_img_height", img_height);
    nh.getParam("img_topic", img_topic);
    nh.getParam("frame_id", frame_id);
    
    camera::Camera MVS_cap(nh);
    image_transport::ImageTransport cam_image(nh);
    image_transport::CameraPublisher img_pub = cam_image.advertiseCamera(img_topic, 1);

    sensor_msgs::Image image_msg;
    sensor_msgs::CameraInfo camera_info_msg;
    
    cv_bridge::CvImagePtr cv_ptr = boost::make_shared<cv_bridge::CvImage>();
    cv_ptr->encoding = sensor_msgs::image_encodings::RGB8;
    ros::Rate loop_rate(30);
    
    while (nh.ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
        MVS_cap.ReadImg(src);
        if (src.empty())
        {
            cout << "empty" << endl;
            continue;
        }else{
            cout << "not empty" <<endl;
            resize(src, resize_src, Size(img_width, img_height));
        }
        cv_ptr->image = resize_src;
        image_msg = *(cv_ptr->toImageMsg());
        image_msg.header.stamp = ros::Time::now();
        image_msg.header.frame_id = frame_id;
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        img_pub.publish(image_msg, camera_info_msg);
    }

    return 0;
}
