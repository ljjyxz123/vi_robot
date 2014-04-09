#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>

#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

VideoWriter depthWriter;
Size depthSize(640, 480);

string face_cascade_name = "haarcascade_frontalface_alt.xml";
CascadeClassifier face_cascade;
string window_name = "人脸识别";

//void rgbCallback(const sensor_msgs::Image::ConstPtr& msg);
//void depthCallback(const sensor_msgs::Image::ConstPtr& msg);
void detectAndDisplay( Mat frame );

void rgbCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	//convert sensor_msgs/Image to Mat
	cv_bridge::CvImagePtr cvImgPtr;
	Mat rgbImg;
	try
	{
		cvImgPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		rgbImg = cvImgPtr->image;
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
	}
	
	int rows = rgbImg.rows;
	int cols = rgbImg.cols;

	try
	{
		//imshow("Get RGBD Image", rgbImg);
		if(!face_cascade.load( face_cascade_name ) )
		{
			printf("[error] 无法加载级联分类器文件！\n");
		}
		
	    detectAndDisplay(rgbImg);
		waitKey(1);
	}
	catch (Exception& e)
	{
		ROS_ERROR("GetRGBDImage rgbCallback exception: %s", e.what());
	}
}

void depthCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	//convert sensor_msgs/Image to Mat
	cv_bridge::CvImagePtr cvImgPtr;
	Mat depthImg;
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
		imshow("Get Depth Image", depthImg);
		if(!face_cascade.load( face_cascade_name ) )
		{
			printf("[error] 无法加载级联分类器文件！\n");
		}
		
		waitKey(1);
	}
	catch (Exception& e)
	{
		ROS_ERROR("GetDepthImage depthCallback exception: %s", e.what());
	}
}

void detectAndDisplay( Mat frame )
{
	std::vector<Rect> faces;
	Mat frame_gray;

	cvtColor( frame, frame_gray, CV_BGR2GRAY );
	equalizeHist( frame_gray, frame_gray );

    face_cascade.detectMultiScale( frame_gray, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, Size(30, 30) );

	ROS_INFO("The number of the faces detected is : %d\n", faces.size());

    for( int i = 0; i < faces.size(); i++ )
	{
		Point center( faces[i].x + faces[i].width*0.5, faces[i].y + faces[i].height*0.5 );
	    ellipse( frame, center, Size( faces[i].width*0.5, faces[i].height*0.5), 0, 0, 360, Scalar( 255, 0, 255 ), 4, 8, 0 );
    }

	//Add Text Information
	int baseLine = depthSize.height - 200;
	int textLeftX = 10;
	int textRightX = depthSize.width - 180;
	int textCenterX = depthSize.width / 2 - 80;
	int lineHeight = 20;
	float frontSize = 0.5;
	Point frontCenter(textCenterX, baseLine);
	Point leftCenter(textLeftX, baseLine);
	Point rightCenter(textRightX, baseLine);
	Scalar textColor(0, 0, 255);
	string text("time: ");
	text.append(boost::lexical_cast<string>(ros::Time::now()));
	putText(frame, text, Point(20, 40), FONT_HERSHEY_SIMPLEX, frontSize, textColor);

    imshow( window_name, frame );
	waitKey(1);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "GetImageFromRGBD");
	ros::NodeHandle node;
	ros::Subscriber depthRawSub = node.subscribe("/camera/depth/image_raw", 1, depthCallback);
	ROS_INFO("begin getting camera depth image");
	ros::Subscriber RgbRawSub = node.subscribe("/camera/rgb/image_raw", 1, rgbCallback);
	ROS_INFO("begin getting camera rgb image");

	ros::spin();

	return 0;
}
