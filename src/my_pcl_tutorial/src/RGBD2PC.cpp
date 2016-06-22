#include <iostream>
#include <string>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
//#include "mini/naviLoc.h"
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/icp.h>
//#include <pcl/registration/impl/icp_nl.hpp>
//#include <pcl/registration/impl/icp.hpp>
#include <pcl/visualization/pcl_visualizer.h>
//#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>

#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

// opencv specific includes
#include <cv_bridge/cv_bridge.h>
#include <cv.h>
#include<highgui.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>


using namespace sensor_msgs;
using namespace message_filters;

#define BOUND(x,min,max)	((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))


typedef pcl::PointXYZRGBA PointType;

class RGBD
{
private:
  int width,height;
  cv::Mat K;
  cv::Mat inv_K;
  cv::Mat M_c2v;

  cv::Mat image;
  cv::Mat depth;
  bool infoReceived, imageReceived, depthReceived;

  cv::Mat xy1;

public:
  const static double Min_Dist = 0.4;
  const static double Max_Dist = 6.0;
  const static double Depth_Scale = 0.001;

  RGBD();

  void updateDepthImage(cv::Mat _depth);
  void updateColorImage(cv::Mat _image);
  void updateCameraInfo(cv::Mat _K, int _width, int _height);
  bool generatePointCloud();

};

class MyPointCloud
{
private:
  bool preLoadCloud;
  pcl::PointCloud<PointType>::Ptr cloud_all_ptr;
  pcl::PointCloud<PointType>::Ptr cloud_now_ptr;
  Eigen::Matrix4f transOdom, transOdomFlag, transOffset;
  pcl::IterativeClosestPoint<PointType, PointType> icp;
  const static int icpFramesToResetTarget = 10000;
  int icpCount;
  bool initialized;
  bool dataReceived;

  void downSamplePoints(pcl::PointCloud<PointType>::Ptr cloud_ptr, 
                        const double& vx=0.02, const double& vy=0.02, const double& vz=0.02);
  void updateRGBAWithHeight(pcl::PointCloud<PointType>::Ptr cloud_ptr,
                        const double& max_height=1.2, const double& min_height = 0.0);
  void removeGroundPoints(pcl::PointCloud<PointType>::Ptr cloud_ptr,
                        const double& min_z=0.1, const double& max_z=1.4);
  void init();
  void loadPCD(std::string s);
public:
  MyPointCloud();
  MyPointCloud(std::string s);
  void updateTrans(const double& x, const double& y, const double& theta);
  void updateTrans(const Eigen::Matrix4f& _trans);
  void updatePoints(pcl::PointCloud<PointType>::Ptr _cloud);
  void visualization();
  void icpStart();
  void icpProcess();

};


// static elements
ros::Publisher pub;
RGBD rgbd_image;
MyPointCloud myPointCloud("data/2015-08-27-21-11-17_pcd.pcd");
boost::mutex io_mutex;

RGBD::RGBD()
{
  infoReceived = false;
  imageReceived = false;
  depthReceived = false;


  // initialed coordinate translate params: y' = z; z' = -y;
  // M = [1 0 0 0; 0 0 1 0; 0 -1 0 0; 0 0 0 1];
  // rotate M in x coordinate  
  
  M_c2v = cv::Mat::eye(4,4,CV_64F);
  M_c2v.at<double>(1,1) = 0;
  M_c2v.at<double>(1,2) = 1;
  M_c2v.at<double>(2,1) = -1;
  M_c2v.at<double>(2,2) = 0;

  double theta = M_PI/2 - 0.1;
  M_c2v.at<double>(1,1) = cos(theta);
  M_c2v.at<double>(1,2) = sin(theta);
  M_c2v.at<double>(2,1) = -sin(theta);
  M_c2v.at<double>(2,2) = cos(theta);
  
  M_c2v.at<double>(0,3) = -0.025;
  M_c2v.at<double>(1,3) = 0.125;
  M_c2v.at<double>(2,3) = 0.23;

  // new_x = z, new_y = -x, new_z = -y;
  M_c2v = cv::Mat::eye(4,4,CV_64F);
  M_c2v.at<double>(0,0) = 0;
  M_c2v.at<double>(1,1) = 0;
  M_c2v.at<double>(2,2) = 0;
  M_c2v.at<double>(0,2) = 1;
  M_c2v.at<double>(1,0) = -1;
  M_c2v.at<double>(2,1) = -1;  
}

bool RGBD::generatePointCloud()
{
  if (!infoReceived)
    return false;
  if (!imageReceived)
    return false;
  if (!depthReceived)
    return false;
  
  // Do generation
  pcl::PointCloud<PointType>::Ptr source_cloud (new pcl::PointCloud<PointType>());
  pcl::PointCloud<PointType> &cloud = *source_cloud;

  //pcl::PointCloud<PointType> cloud;
  cloud.width = width*height;
  cloud.height = 1;
  cloud.points.resize(cloud.width*cloud.height);

  double nowDepth = 0;
  int num = 0;

  for (int i=0; i<width; i++)
  {
    for (int j=0; j<height; j++)
    {
      nowDepth = depth.at<unsigned short>(j,i) * Depth_Scale;
      //nowDepth = 1.0;

      if (nowDepth < Min_Dist || nowDepth > Max_Dist)
        continue;
      
      cloud.points[num].x = nowDepth * xy1.at<double>(0, i*height+j);
      cloud.points[num].y = nowDepth * xy1.at<double>(1, i*height+j);
      cloud.points[num].z = nowDepth;
      
      cv::Vec3b& elem = image.at<cv::Vec3b>(j,i);


      cloud.points[num].rgba = 0;
      cloud.points[num].rgba = (cloud.points[num].rgba<<8) + elem[2];
      cloud.points[num].rgba = (cloud.points[num].rgba<<8) + elem[1];
      cloud.points[num].rgba = (cloud.points[num].rgba<<8) + elem[0];
      num++;
    }
  }
  cloud.width = num;
  cloud.points.resize(cloud.width*cloud.height);
  //ROS_INFO("%d",num);

  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  for (int i=0; i<4; i++)
    for (int j=0; j<4; j++)
      transform(i,j) = M_c2v.at<double>(i,j);

  pcl::PointCloud<PointType>::Ptr transformed_cloud (new pcl::PointCloud<PointType>());
  pcl::transformPointCloud (*source_cloud, *transformed_cloud, transform);

  myPointCloud.updatePoints(transformed_cloud);
  
  // set false  
  imageReceived = false;
  depthReceived = false;

  return true;
}

void RGBD::updateDepthImage(cv::Mat _depth)
{
  depth = _depth;
  depthReceived = true;
  generatePointCloud();
}

void RGBD::updateColorImage(cv::Mat _image)
{
  image = _image;
  imageReceived = true;
  generatePointCloud();
}

void RGBD::updateCameraInfo(cv::Mat _K, int _width, int _height)
{
  K = _K;
  width = _width;
  height = _height;
  infoReceived = true;

  inv_K = K.inv();
  //ROS_INFO("%f, %f, %f\n %f, %f, %f\n %f, %f %f",
  //  K.at<double>(0,0), K.at<double>(0,1), K.at<double>(0,2),
  //  K.at<double>(1,0), K.at<double>(1,1), K.at<double>(1,2),
  //  K.at<double>(2,0), K.at<double>(2,1), K.at<double>(2,2));

  cv::Mat uv1(3, width*height, CV_64F, 1.0);
  for (int i=0; i<width; i++)
    for (int j=0; j<height; j++)
    {
      uv1.at<double>(0,i*height+j) = i;
      uv1.at<double>(1,i*height+j) = j;
      uv1.at<double>(2,i*height+j) = 1.0;
    }

  xy1 = inv_K*uv1;
}


void MyPointCloud::downSamplePoints(pcl::PointCloud<PointType>::Ptr cloud_ptr, 
                        const double& vx, const double& vy, const double& vz)
{
  pcl::PointCloud<PointType> cloud_filtered;
  pcl::VoxelGrid<PointType> sor;
  sor.setInputCloud (cloud_ptr);
  sor.setLeafSize (vx, vy, vz);
  sor.filter (cloud_filtered);
  *cloud_ptr = cloud_filtered;
}

void MyPointCloud::updateRGBAWithHeight(pcl::PointCloud<PointType>::Ptr cloud_ptr,
                        const double& max_height, const double& min_height)
{
  int r=0,g=0,b=0,rgba=0;
  double scale = 256.0/(max_height-min_height);

  //ROS_INFO("%f",cloud_ptr->points[0].z);

  for (int i=0; i<cloud_ptr->points.size(); i++)
  {
    r = BOUND((cloud_ptr->points[i].z-min_height)*scale,0,255);
    //r = 255;
    b = 255-r;
    if (r<128)
      g = (r)*2;
    else
      g = (b)*2;
    rgba = (r<<16) + (g<<8) + b;
    cloud_ptr->points[i].rgba = rgba;
  }
}

void MyPointCloud::removeGroundPoints(pcl::PointCloud<PointType>::Ptr cloud_ptr,
                        const double& min_z, const double& max_z)
{
  pcl::PointCloud<PointType> cloud_filtered;

  pcl::PassThrough<PointType> pass;
  pass.setInputCloud(cloud_ptr);
  pass.setFilterFieldName("z");
  pass.setFilterLimits(min_z,max_z);
  pass.setFilterLimitsNegative(false);
  pass.filter(cloud_filtered);
  *cloud_ptr = cloud_filtered;
}

void MyPointCloud::init()
{
  transOdom = Eigen::Matrix4f::Identity();
  transOdomFlag = Eigen::Matrix4f::Identity();
  transOffset = Eigen::Matrix4f::Identity();  
  pcl::PointCloud<PointType>::Ptr tmp_cloud(new pcl::PointCloud<PointType>);  
  cloud_all_ptr = tmp_cloud;
  icpCount = 0;
  initialized = false;
  dataReceived = false;
  preLoadCloud = false;
}

void MyPointCloud::loadPCD(std::string s)
{
  if (pcl::io::loadPCDFile<PointType>(s,*cloud_all_ptr)!=-1)
  {
    std::cout << "PCD Load Succeed: Size " << cloud_all_ptr->width*cloud_all_ptr->height << std::endl;
    downSamplePoints(cloud_all_ptr);
    removeGroundPoints(cloud_all_ptr);
    //updateRGBAWithHeight(cloud_all_ptr);
    preLoadCloud = true;
  }else
    std::cout <<"PCD Load Failed" << std::endl;
}

MyPointCloud::MyPointCloud()
{
  init();
  boost::thread thrd1(boost::bind(&MyPointCloud::visualization,this));
  boost::thread thrd2(boost::bind(&MyPointCloud::icpStart,this));
}

MyPointCloud::MyPointCloud(std::string s)
{
  init();
  loadPCD(s);
  boost::thread thrd1(boost::bind(&MyPointCloud::visualization,this));
  boost::thread thrd2(boost::bind(&MyPointCloud::icpStart,this));
}

void MyPointCloud::visualization()
{

  int count = 0;

  while (!initialized)
  {
    //std::cout << count++ << std::endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

  pcl::PointCloud<PointType>::Ptr cloud_view_all_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_view_now_ptr(new pcl::PointCloud<PointType>);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  //transform(2,2) = 0;

  {
    boost::mutex::scoped_lock lock(io_mutex);
    pcl::transformPointCloud(*cloud_all_ptr, *cloud_view_all_ptr, transform);
  }
  //downSamplePoints(cloud_view_all_ptr);

  Eigen::Matrix4f preTransform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f _transOffset, _transOdom, _transOdomFlag;


  pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
  bool visualInitialed = false;
  viewer.initCameraParameters ();
  while (!viewer.wasStopped())
  {

    {
      boost::mutex::scoped_lock lock(io_mutex);
      _transOffset = this->transOffset;
      _transOdomFlag = this->transOdomFlag;
      _transOdom = this->transOdom;
      pcl::transformPointCloud(*cloud_now_ptr, *cloud_view_now_ptr, transform * _transOffset * _transOdomFlag);
      if (!preLoadCloud)
        pcl::transformPointCloud(*cloud_all_ptr, *cloud_view_all_ptr, transform);
    }
    //updateRGBAWithHeight(cloud_view_all_ptr);
  
    //if (!preLoadCloud)
    //  downSamplePoints(cloud_view_all_ptr);
    //downSamplePoints(cloud_view_now_ptr);

    if (!visualInitialed)
    {
      //pcl::visualization::PointCloudColorHandlerCustom<PointType> now_color_handler (cloud_view_now_ptr, 230, 20, 20);
      
      //updateRGBAWithHeight(cloud_view_all_ptr);
      viewer.addPointCloud (cloud_view_all_ptr, "original_cloud");
      updateRGBAWithHeight(cloud_view_now_ptr);  
      viewer.addPointCloud (cloud_view_now_ptr, "tranformed_cloud");
      Eigen::Transform<float, 3, Eigen::Affine> t_src2w (_transOdom);
      viewer.addCoordinateSystem (0.2, t_src2w);
      viewer.setBackgroundColor(0.1, 0.1, 0.1, 0);
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "original_cloud");
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tranformed_cloud");
      visualInitialed = true;
    }else
    {
      if (!preLoadCloud)
        viewer.updatePointCloud (cloud_view_all_ptr, "original_cloud");
      updateRGBAWithHeight(cloud_view_now_ptr);  
      viewer.updatePointCloud (cloud_view_now_ptr, "tranformed_cloud");    
      
      if (_transOdom != preTransform)
      {
        preTransform = _transOdom;
        Eigen::Transform<float, 3, Eigen::Affine> t_src2w (_transOffset*_transOdom);
        viewer.addCoordinateSystem (0.2, t_src2w);
      }
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "tranformed_cloud");
    }
    //std::cout << count++ << std::endl;
    viewer.spinOnce (100);
  }
}


void MyPointCloud::updateTrans(const double& x, const double& y, const double& theta)
{
  transOdom(0,0) = cos(theta);
  transOdom(0,1) = -sin(theta);
  transOdom(1,0) = sin(theta);
  transOdom(1,1) = cos(theta);
  transOdom(0,3) = x;
  transOdom(1,3) = y;
}

void MyPointCloud::updateTrans(const Eigen::Matrix4f& _trans)
{
  transOdom = _trans;
}

void MyPointCloud::icpStart()
{
  pcl::PointCloud<PointType>::Ptr cloud_icp_all_ptr(new pcl::PointCloud<PointType>);
  pcl::PointCloud<PointType>::Ptr cloud_icp_now_ptr(new pcl::PointCloud<PointType>);
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f _transOffset, _transOdomFlag;

  if (preLoadCloud)
  {
    {
      boost::mutex::scoped_lock lock(io_mutex);
      pcl::transformPointCloud(*cloud_all_ptr, *cloud_icp_all_ptr, transform);  
    }
    downSamplePoints(cloud_icp_all_ptr);
    removeGroundPoints(cloud_icp_all_ptr);
    icp.setInputTarget(cloud_icp_all_ptr);
  }

  icp.setMaxCorrespondenceDistance(0.1);
  // Set the maximum number of iterations (criterion 1)
  icp.setMaximumIterations(50);
  // Set the transformation epsilon (criterion 2)
  icp.setTransformationEpsilon(1e-8);
  // Set the euclidean distance difference epsilon (criterion 3)
  //icp.setEuclideanFitnessEpsilon(1);
  
  int count = 0;
  while (!initialized)
  {
    std::cout << "icp wait  " << count++ << std::endl;
    boost::this_thread::sleep(boost::posix_time::milliseconds(100));
  }

  
  while(true)
  {
    if (!dataReceived)
    {
      boost::this_thread::sleep(boost::posix_time::milliseconds(100));
      continue;
    }

    {   //copy data
      boost::mutex::scoped_lock lock(io_mutex);
      _transOffset = this->transOffset;
      _transOdomFlag = this->transOdomFlag;
      if (!preLoadCloud)       
        pcl::transformPointCloud(*cloud_all_ptr, *cloud_icp_all_ptr, transform);
      pcl::transformPointCloud(*cloud_now_ptr, *cloud_icp_now_ptr, _transOffset*_transOdomFlag);
      dataReceived = false;
    }

    if (!preLoadCloud)
      icp.setInputTarget(cloud_icp_all_ptr);

    downSamplePoints(cloud_icp_now_ptr);
    removeGroundPoints(cloud_icp_now_ptr);
    icp.setInputSource(cloud_icp_now_ptr);
    pcl::PointCloud<PointType> cloud_registered;
    icp.align(cloud_registered);

    Eigen::Matrix4f _transOffset2 = icp.getFinalTransformation();
    _transOffset = _transOffset2 * _transOffset;
    std::cout<< "has convergerd: " << icp.hasConverged() << "score: " << icp.getFitnessScore() << std::endl;
    std::cout<<_transOffset<<std::endl;
    //std::cout<<icp.getFinalTransformation()<<std::endl;

    {
      boost::mutex::scoped_lock lock(io_mutex);
      this->transOffset = _transOffset;
    }
  }

  // do icp
  
}

void MyPointCloud::updatePoints(pcl::PointCloud<PointType>::Ptr _cloud)
{
  downSamplePoints(_cloud);
  removeGroundPoints(_cloud);
  //updateRGBAWithHeight(_cloud,-100, -100.1);
  {
    boost::mutex::scoped_lock lock(io_mutex);
    cloud_now_ptr = _cloud;
    transOdomFlag = transOdom;
    if (!preLoadCloud)
    {
      pcl::PointCloud<PointType>::Ptr cloud_transformed_ptr(new pcl::PointCloud<PointType>());
      pcl::transformPointCloud(*cloud_now_ptr, *cloud_transformed_ptr, transOffset*transOdom);
      *cloud_all_ptr += *cloud_transformed_ptr;
      downSamplePoints(cloud_all_ptr);
    }
  }
  dataReceived = true;
  initialized = true;

  /*
  // transform cloud_now
  pcl::PointCloud<PointType>::Ptr cloud_transformed_ptr(new pcl::PointCloud<PointType>());
  Eigen::Matrix4f transform = transOdom*transOffset;
  pcl::transformPointCloud(*_cloud, *cloud_transformed_ptr, transform);

  downSamplePoints(cloud_transformed_ptr);
  removeGroundPoints(cloud_transformed_ptr);


  // icp
  icpCount++;
  if (initialized)
  {
    icp.setInputSource(cloud_transformed_ptr);
    //if (icpCount%icpFramesToResetTarget==0)
    //    icp.setInputTarget(cloud_all_ptr);
    // Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance(0.05);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations(50);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon(1e-8);
    // Set the euclidean distance difference epsilon (criterion 3)
    //icp.setEuclideanFitnessEpsilon(1);
    pcl::PointCloud<PointType> cloud_registered;
    icp.align(cloud_registered);

    Eigen::Matrix4f _transOffset = icp.getFinalTransformation();
    transOffset = _transOffset*transOffset;
    
    std::cout<< "has convergerd: " << icp.hasConverged() << "score: " << icp.getFitnessScore() << std::endl;
    std::cout<<transOffset<<std::endl;
    //std::cout<<icp.getFinalTransformation()<<std::endl;
    *cloud_transformed_ptr = cloud_registered;
    
    //if (!preLoadCloud)
    //  *cloud_all_ptr += cloud_registered;
  }else
  {
    if (!preLoadCloud)
      *cloud_all_ptr += *cloud_transformed_ptr;
    icp.setInputTarget(cloud_all_ptr);
  }

  if (!preLoadCloud || !initialized)
  {    
    downSamplePoints(cloud_all_ptr);//,0.1,0.1,0.1);
    removeGroundPoints(cloud_all_ptr);
  }


  updateRGBAWithHeight(cloud_transformed_ptr,-100,-100.1);
  //updateRGBAWithHeight(cloud_all_ptr, 100.1, 100);

  cloud_now_ptr = cloud_transformed_ptr;

  //ROS_INFO("%d", cloud_all_ptr->width);
  */
}

void init()
{
  //cv::namedWindow("tmp");
}

void depth_cb(const sensor_msgs::ImageConstPtr& depth_image_msg)
{
  cv::Mat depth_image =  cv_bridge::toCvCopy(depth_image_msg)->image;
  rgbd_image.updateDepthImage(depth_image);

  //ROS_INFO("depth: %d.%d",depth_image_msg->header.stamp.sec, depth_image_msg->header.stamp.nsec);
}

void image_cb(const sensor_msgs::ImageConstPtr& color_image_msg)
{
  cv::Mat color_image =  cv_bridge::toCvCopy(color_image_msg)->image;
  rgbd_image.updateColorImage(color_image);

  //ROS_INFO("image: %d.%d",color_image_msg->header.stamp.sec, color_image_msg->header.stamp.nsec);
//  cv::imshow("tmp",color_image);
//  cv::waitKey(10);
}


void info_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  cv::Mat K;
  K = cv::Mat::eye(3,3,CV_64F);
  for (int i=0; i<3; i++)
    for (int j=0; j<3; j++)
      K.at<double>(i,j) = info_msg->K[i*3+j];

  rgbd_image.updateCameraInfo(K, info_msg->width, info_msg->height);

  //ROS_INFO("info: %d.%d",info_msg->header.stamp.sec, info_msg->header.stamp.nsec);
}

//void nav_cb(const mini::naviLoc::ConstPtr& msg)
//{
  //ROS_INFO("%f,%f,%f",msg->x,msg->y,msg->theta);
  //myPointCloud.updateTrans(msg->x,msg->y,msg->theta);
//}

void listeningTf()
{
  int count = 0;
  tf::TransformListener listener;
  ros::Rate rate(100.0);
  while (1){
    tf::StampedTransform transform;
    try{
      ros::Time now = ros::Time::now();
      listener.waitForTransform("/odom_frame", "/base_frame",
                              ros::Time(0), ros::Duration(3.0));
      listener.lookupTransform("/odom_frame", "/base_frame",
                             ros::Time(0), transform);
    }
    catch (tf::TransformException &ex) {
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    //std::cout << ++count << std::endl;
    
    Eigen::Affine3d matrix;
    tf::transformTFToEigen (transform, matrix);
    Eigen::Matrix4d transEigenD = matrix.matrix();
    Eigen::Matrix4f transEigenF;
    for (int i=0; i<4; i++)
      for (int j=0; j<4; j++)
        transEigenF(i,j) = transEigenD(i,j);
    Eigen::Matrix4f refineM = transEigenF;
    //refineM(0,0) = 0;
    //refineM(0,1) = -1;
    //refineM(1,0) = 1;
    //refineM(1,1) = 0;
    refineM(0,3) = transEigenF(1,3);
    refineM(1,3) = -transEigenF(0,3);
    {
      boost::mutex::scoped_lock lock(io_mutex);
      myPointCloud.updateTrans(refineM);
      //std::cout << "tf" << std::endl;
    }
    rate.sleep();
  }
}

int main(int argc, char** argv)
{
  
  init();

  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber depth_sub = nh.subscribe<sensor_msgs::Image> ("/camera/depth_registered/image_raw", 1, depth_cb);
  ros::Subscriber image_sub = nh.subscribe<sensor_msgs::Image> ("/camera/rgb/image_color", 1, image_cb);
  ros::Subscriber info_sub = nh.subscribe<sensor_msgs::CameraInfo> ("/camera/rgb/camera_info", 1, info_cb);
  //ros::Subscriber nav_sub = nh.subscribe<mini::naviLoc> ("navmsg", 1, nav_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("/down_sampled_cloud", 1);

  boost::thread thrdListeningTf(&listeningTf);

  ros::spin();

  return 0;
}
