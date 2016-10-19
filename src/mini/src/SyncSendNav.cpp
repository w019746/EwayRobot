#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sys/time.h>  
#include <time.h> 
#include <tf/transform_broadcaster.h>
#include <vector>
#include <sensor_msgs/CameraInfo.h>

using namespace std;

const double PI = 3.1415926535897932384626433832795;
static vector<vector<double> > navData;
static int cursor = 0;
static tf::Transform transform_v2c,transform_w2v;
static tf::TransformBroadcaster *pbr = 0;

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

void FindFrameAndSendTF(const sensor_msgs::ImageConstPtr& color_image_msg)
{
  //tf::TransformBroadcaster br;
  unsigned int timeSec = color_image_msg->header.stamp.sec;
  unsigned int timeNSec = color_image_msg->header.stamp.nsec;
  double timeCombine = double(timeSec)+ double(timeNSec/10000000)/100;
  
  cameraInfo.header = color_image_msg->header;
  cameraInfo.width = color_image_msg->width;
  cameraInfo.height = color_image_msg->height;
  //ROS_INFO("image: %d.%d %s",color_image_msg->header.stamp.sec, color_image_msg->header.stamp.nsec, color_image_msg->encoding.c_str());
  //ROS_INFO("width: %d, height: %d", color_image_msg->width, color_image_msg->height);
  pub.publish(cameraInfo);

  if(navData[cursor][0]<timeCombine)
  {
    //选取一帧nav数据
    for(;cursor<navData.size() && navData[cursor][0]<timeCombine; cursor++){}
    
    if(navData[cursor][0]>=timeCombine)
    {
      //转化成tf并发送
      tf::Quaternion q;
      ros::Time ros_time(navData[cursor][0]);
      //printf("%f,%lu\n",ros_time.toSec(),ros_time.toNSec());
		  transform_w2v.setOrigin(tf::Vector3(navData[cursor][1],navData[cursor][2],0.0)); 
		  q.setRPY(0.0,0.0,navData[cursor][3]);
		  transform_w2v.setRotation(q);
		  transform_w2v = transform_w2v.inverse();

      cout << "send OK!" << endl;
      pbr->sendTransform(tf::StampedTransform(transform_w2v,ros_time, "/base_frame","/odom_frame"));
		  pbr->sendTransform(tf::StampedTransform(transform_v2c,ros_time, "/camera_rgb_optical_frame","/base_frame"));
    }
  }
}

int main(int argc, char** argv)
{
  initInfo();
  //read nav file
  FILE* fp = fopen("data/20160523/nav_20160523_1.txt","r");
  vector<double> temp(4);  
  while(fscanf(fp,"%lf %lf %lf %lf",&temp[0], &temp[1], &temp[2], &temp[3])>0)
  {
    double ttemp = temp[2];
    temp[2] = temp[1];
    temp[1] = -ttemp;
    temp[3] = temp[3]/180*PI;
    navData.push_back(temp);
  }

  tf::Quaternion q(-sqrt(2)/2,0,0,sqrt(2)/2); //need calibration
  //transform_v2c.setOrigin(tf::Vector3(0.027,0.03,0.523)); //need calibration
	transform_v2c.setOrigin(tf::Vector3(0.01,-0.03,0.844)); 
  transform_v2c.setRotation(q);	
  transform_v2c = transform_v2c.inverse();

	ros::init(argc, argv, "SyncSendNav");
  ros::NodeHandle n;
  
  pbr = new tf::TransformBroadcaster;

  //接收有时间戳的消息
  ros::Subscriber sub = n.subscribe("/camera/rgb/image_color",1,FindFrameAndSendTF);

  pub = n.advertise<sensor_msgs::CameraInfo> ("/camera/rgb/camera_info", 1);

	ros::spin();
  fclose(fp);
  delete pbr;
	return 0;
}

