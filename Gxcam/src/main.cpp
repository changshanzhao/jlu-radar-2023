#include "GxCamera.h"

#include"opencv2/opencv.hpp"
#include"math.h"
#include"string"
#include <vector>  
#include <algorithm>  
#include <iostream>  
#include <iterator>  
#include <stdio.h>  
#include <stdlib.h>  
#include <ctype.h>   

#include "ros/ros.h"
#include  "sensor_msgs/Image.h"


#include "sensor_msgs/CompressedImage.h"
#include"sensor_msgs/CameraInfo.h"
#include "cv_bridge/cv_bridge.h"
#include"tf/transform_broadcaster.h"

sensor_msgs::CameraInfo getCameraInfo(void){        // extract cameraInfo. signal right
    
	sensor_msgs::CameraInfo cam;
    vector<double> D{-0.098478, 0.023177, -0.000431, -0.000113, 0.000000};
    boost::array<double, 9> K = {
      653.47554,   0.     , 950.58212,
           0.     , 595.02553, 669.88204,
           0.     ,   0.     ,   1.
    };
	boost::array<double, 12> P = {
      623.17963,   0.     , 951.633  ,   0.     ,
           0.     , 565.21637, 664.70577,   0.     ,
           0.     ,   0.     ,   1.     ,   0.
    };
    boost::array<double, 9> r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};

    cam.width = 1920;
    cam.height = 1280;
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

int main(int argc, char* argv[]) {
	//std::cout<<"0"<<std::endl;
	ros::init(argc,argv,"Gxcam_node");
	ros::NodeHandle nh("~");
	ros::Publisher cam_img_pub = nh.advertise<sensor_msgs::Image>("cam_image",1);
	ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("cam_info",1);
	sensor_msgs::ImagePtr img_msg;
	sensor_msgs::CameraInfo cam_info = getCameraInfo();
	ros::Rate r(60);
#pragma region cam1
	GX_DEV_HANDLE camHandle_1 = NULL;
	GX_STATUS status = GX_STATUS_SUCCESS;
	/*
	 *Preparation: CvMat image content
	*/
	Mat frame;
	/*
	 *First init: Implementation of GxCamera and init it
	*/
	GxCamera gxCam;
	status = gxCam.initLib();
	GX_VERIFY(status);
	std::cout<<"28"<<std::endl;
	/*
	 *Second init: Open Camera by SN/Index
	*/
	status = gxCam.openDeviceBySN("FCM23020072",camHandle_1);	//By SN 
	//status = gxCam.openDeviceByIndex("0",camHandle_1);			//By Index
	GX_VERIFY(status);
	/*
	 *Third init: Set Camera Params: ROI, Exposure, Gain, WhiteBalance
	*/
	std::cout<<"29"<<std::endl;
	gxCam.setRoiParam(2048, 1536, 0, 0);				// ROI
	std::cout<<"30"<<std::endl;
	gxCam.setExposureParam(25000, true, 100, 30000);	// Exposure
	std::cout<<"31"<<std::endl;
	gxCam.setGainParam(5, true, 0, 10);					// Gain
	std::cout<<"32"<<std::endl;
	gxCam.setWhiteBalanceOn(true,camHandle_1);						// WhiteBalance
	std::cout<<"33"<<std::endl;
	gxCam.setTriggerGrab(camHandle_1,false);				//Trigger
	std::cout<<"34"<<std::endl;
	/*
	 *Before acq: Send Acquisition Start Command
	*/
	
	status = gxCam.startAcquiring(camHandle_1);					// Send Start Acquisition Command
	
	GX_VERIFY(status);
	std::cout<<"35"<<std::endl;
#pragma endregion

// #pragma region  cam2
// 	GX_DEV_HANDLE camHandle_2 = NULL;
// 	GX_STATUS status2 = GX_STATUS_SUCCESS;
// 	/*
// 	 *Preparation: CvMat image content
// 	*/
// 	Mat frame2;
// 	/*
// 	 *First init: Implementation of GxCamera and init it
// 	*/
// 	GxCamera gxCam2;
// 	status2 = gxCam2.initLib();
// 	GX_VERIFY(status2);
// 	/*
// 	 *Second init: Open Camera by SN/Index
// 	*/
// 	status2 = gxCam2.openDeviceBySN("GCA22040080",camHandle_2);	//By SN
// 	//  status = gxCam.openDeviceByIndex("2",camHandle_1);			//By Index
// 	GX_VERIFY(status2);
// 	/*
// 	 *Third init: Set Camera Params: ROI, Exposure, Gain, WhiteBalance
// 	*/
// 	gxCam2.setRoiParam(4096, 3000, 0, 0);				// ROI
// 	gxCam2.setExposureParam(25000, false, 1000, 30000);	// Exposure
// 	gxCam2.setGainParam(5, false, 0, 10);					// Gain
// 	gxCam2.setWhiteBalanceOn(true,camHandle_2);						// WhiteBalance
// 	gxCam2.setTriggerGrab(camHandle_2);		;		//Trigger
// 	/*
// 	 *Before acq: Send Acquisition Start Command
// 	*/
// 	status2 = gxCam2.startAcquiring(camHandle_2);					// Send Start Acquisition Command
// 	GX_VERIFY(status2);
// #pragma endregion
	std::cout<<"36"<<std::endl;

	int i = 1;
	while (ros::ok())
	{
		/*
		 *In acq: Snap a CvMat Image and store it in CvMat Content
		*/
		std::cout<<"37"<<std::endl;
		status = gxCam.snapCvMat(frame,camHandle_1);				// Snap an image and return as CvMat Foramt
		GX_VERIFY(status);
		

		// status2 = gxCam2.snapCvMat(frame2,camHandle_2);				// Snap an image and return as CvMat Foramt
		// GX_VERIFY(status2);

		//resize(frame,frame,Size(1080,720));
		// resize(frame2,frame2,Size(640,480));
		std::cout<<"38"<<std::endl;
		// imshow("src",frame);
		// waitKey(1);
		// gxCam.printFrameInfo();
		// gxCam2.printFrameInfo();
		std_msgs::Header h;
		h.stamp = ros::Time::now();
		h.frame_id = "camera";
		img_msg = cv_bridge::CvImage(h,"bgr8",frame).toImageMsg();
		std::cout<<"39"<<std::endl;
		cam_img_pub.publish(*img_msg);
		cam_info_pub.publish(cam_info);
		std::cout<<"40"<<std::endl;
		ros::spinOnce();
		r.sleep();

		//cvNamedWindow("left",CV_WINDOW_NORMAL);
		//imshow("left", frame);
		// cvNamedWindow("right",CV_WINDOW_AUTOSIZE);
		// imshow("right", frame2);
		//char chKey = waitKey(1);
		//if (chKey == 's') {
			//cv::imwrite("/home/robot/campus/calibration/r3live/"+to_string(i)+".jpg",frame);
			// cv::imwrite("/home/robot/campus/GxCam/right_img/"+to_string(i)+".jpg",frame2);
			//i++;
		//}
		//if (chKey == 27) break;
	}
#pragma region cam1
	/*
	 *After acq: Send Acquisition Stop Command
	*/
	status = gxCam.stopAcquiring(camHandle_1);
	GX_VERIFY(status);
	/*
	*Close camera, while you can still open a new different camera
	*/
	gxCam.closeDevice(camHandle_1);
	/*
	*Close lib: you can not use any GxCamera device unless you initLib() again
	*/
	gxCam.closeLib();
#pragma endregion
// #pragma region cam2
// 	status2 = gxCam2.stopAcquiring(camHandle_2);
// 	GX_VERIFY(status2);
// 	/*
// 	*Close camera, while you can still open a new different camera
// 	*/
// 	gxCam2.closeDevice(camHandle_2);
// 	/*
// 	*Close lib: you can not use any GxCamera device unless you initLib() again
// 	*/
// 	gxCam2.closeLib();
// #pragma endregion
	system("pause");
	return 0;
}
