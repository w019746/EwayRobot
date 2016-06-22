#include <iostream>
#include <string>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

using namespace sensor_msgs;

ros::Publisher pub;
sensor_msgs::CameraInfo cameraInfo;

void initInfo()
{
  cameraInfo.distortion_model = "plumb_bob";
  
  cameraInfo.D.resize(5);
  for (int i=0; i<5; i++)
    cameraInfo.D[i] = 0.0;

  cameraInfo.K[0] = 525.0;  cameraInfo.K[1] = 0.0;  cameraInfo.K[2] = 319.5;  
  cameraInfo.K[3] = 0.0;  cameraInfo.K[4] = 525.0;  cameraInfo.K[5] = 239.5;  
  cameraInfo.K[6] = 0.0;  cameraInfo.K[7] = 0.0;  cameraInfo.K[8] = 1.0;  

  cameraInfo.R[0] = 1.0;  cameraInfo.R[1] = 0;  cameraInfo.R[2] = 0;  
  cameraInfo.R[3] = 0;  cameraInfo.R[4] = 1.0;  cameraInfo.R[5] = 0;  
  cameraInfo.R[6] = 0;  cameraInfo.R[7] = 0;  cameraInfo.R[8] = 1.0;  

  cameraInfo.P[0] = 525.0;  cameraInfo.P[1] = 0.0;  cameraInfo.P[2] = 319.5;  cameraInfo.P[3] = 0.0; 
  cameraInfo.P[4] = 0.0;  cameraInfo.P[5] = 525.0;  cameraInfo.P[6] = 239.5;  cameraInfo.P[7] = 0.0; 
  cameraInfo.P[8] = 0.0;  cameraInfo.P[9] = 0.0;  cameraInfo.P[10] = 1.0;  cameraInfo.P[11] = 0.0; 
}


void image_cb(const sensor_msgs::ImageConstPtr& color_image_msg)
{
  cameraInfo.header = color_image_msg->header;
  cameraInfo.width = color_image_msg->width;
  cameraInfo.height = color_image_msg->height;
  ROS_INFO("image: %d.%d %s",color_image_msg->header.stamp.sec, color_image_msg->header.stamp.nsec, color_image_msg->encoding.c_str());
  ROS_INFO("width: %d, height: %d", color_image_msg->width, color_image_msg->height);
  pub.publish(cameraInfo);
}


int main(int argc, char** argv)
{
  initInfo();
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;

  ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_raw", 1, image_cb);
  
  pub = nh.advertise<sensor_msgs::CameraInfo> ("/camera/rgb/camera_info", 1);
  ros::spin();

  return 0;
}
