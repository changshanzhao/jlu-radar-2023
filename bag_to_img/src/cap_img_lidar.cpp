#include"opencv2/opencv.hpp"
#include"string"
#include "ros/ros.h"
#include  "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include"sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"
#include"tf/transform_broadcaster.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/point_cloud.h"
#include "pcl/io/pcd_io.h"
#include "pcl_conversions/pcl_conversions.h"
#include"message_filters/subscriber.h"
#include"message_filters/time_synchronizer.h"
#include <message_filters/sync_policies/approximate_time.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;

std::string pic_name;
std::string pcd_name;
std::string topic_cam;
std::string topic_lidar;
int i = 0;

void callback(sensor_msgs::Image::ConstPtr img_msg,sensor_msgs::PointCloud2::ConstPtr pcl){
	cv::Mat img;
	cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img_msg,sensor_msgs::image_encodings::TYPE_8UC1);
	img =cv_ptr->image;

    pcl::PointCloud<pcl::PointXYZ> pp;
    pcl::fromROSMsg(*pcl,pp);

	// cv::Mat img_show;
	// cv::resize(img,img_show,cv::Size(640,480));
	cv::imshow("src",img);
	if(cv::waitKey(1)=='s'){
		cv::imwrite("/home/robot/campus/calibration/r3live/4.13_car-realsense/pics/"+std::to_string(i)+".png",img);
        pcl::io::savePCDFile( "/home/robot/campus/calibration/r3live/4.13_car-realsense/pcd/" + std::to_string(i)+".pcd", pp);
		i++;
	}
}

int main(int argc,char** argv){
	cv::namedWindow("src",CV_WINDOW_AUTOSIZE);
	ros::init(argc,argv,"cap_img_lidar_node");
	ros::NodeHandle nh;
	nh.param<std::string>("pic_name", pic_name, "0");
	nh.param<std::string>("topic_cam",topic_cam,"0");
    nh.param<std::string>("pcd_name", pcd_name, "0");
	nh.param<std::string>("topic_lidar",topic_lidar,"0");

	message_filters::Subscriber<sensor_msgs::Image> sub_cam(nh,topic_cam,1);
	message_filters::Subscriber<sensor_msgs::PointCloud2> sub_lidar(nh,topic_lidar,1);
    // message_filters::TimeSynchronizer<sensor_msgs::Image,sensor_msgs::PointCloud2> sync(sub_cam,sub_lidar,10);
    message_filters:: Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub_cam, sub_lidar);
    sync.registerCallback(boost::bind(&callback,_1,_2));

	ros::spin();
	return 0;
}

