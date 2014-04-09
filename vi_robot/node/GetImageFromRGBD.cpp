#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <sstream>

using namespace cv;

void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	//convert sensor_msgs/Image to Mat
	cv_bridge::CvImagePtr cvImgPtr;
	Mat_<uint16_t> depthImg;
	try
	{
		cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
		depthImg = cvImgPtr->image;
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	
	int rows = depthImg.rows;
	int cols = depthImg.cols;

	try
	{
		//imshow("GetRGBDImage", depthColorImg);
		imshow("Get RGBD Image", depthImg);
		waitKey(10);
	}
	catch (Exception& e)
	{
		ROS_ERROR("GetRGBDImage depthCallback exception: %s", e.what());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "GetImageFromRGBD");
	ros::NodeHandle node;
	ros::Subscriber depthRawSub = node.subscribe("/camera/depth/image_raw", 100, depthCallback);
	ROS_INFO("begin getting camera rgb image");

	ros::spin();

	return 0;
}
