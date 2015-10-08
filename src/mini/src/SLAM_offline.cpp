#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


class ImageCollector
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Subscriber depth_sub_;
   
   
public:
	ImageCollector()
	 : it_(nh_)
	{
		image_sub_ = it_.subscribe("/camera/rgb/image_color", 1000, &ImageCollector::imageCb, this);
		depth_sub_ = it_.subscribe("/camera/depth_registered/image_raw", 1000, &ImageCollector::depthCb, this);
	}

	~ImageCollector(){}

	void imageCb(const sensor_msgs::ImageConstPtr& msg)
	{
		
		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvShare(msg);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
	 	}
		char file_name[100];
		int time_stamp = msg->header.stamp.sec%100000*1000+msg->header.stamp.nsec/1000000;
		sprintf(file_name,"/home/haha/catkin_ws/rgb/%d.png",time_stamp);
		cv::imwrite(file_name,cv_ptr->image);
	}

	void depthCb(const sensor_msgs::ImageConstPtr& msg)
	{
		
		cv_bridge::CvImageConstPtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvShare(msg);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
	 	}
		char file_name[100];
		int time_stamp = msg->header.stamp.sec%100000*1000+msg->header.stamp.nsec/1000000;
		sprintf(file_name,"/home/haha/catkin_ws/depth/%d.png",time_stamp);
		cv::imwrite(file_name,cv_ptr->image);
	}
 };
 
int main(int argc, char** argv)
{
	ros::init(argc, argv, "SLAM_offline");
	ImageCollector ic;
	ros::spin();
	return 0;
}


