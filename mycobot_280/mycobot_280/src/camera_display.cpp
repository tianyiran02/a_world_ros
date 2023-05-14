#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
	try
	{
		cv::Mat img = cv_bridge::toCvCopy(msg, "bgr8")->image;

		// Get original dimensions
		int height = img.rows;
		int width = img.cols;

		// Define resize ratio
		double ratio = 0.25;

		// Compute new dimensions
		int new_height = static_cast<int>(height * ratio);
		int new_width = static_cast<int>(width * ratio);

		// Resize image
		cv::Mat resized_img;
    	cv::resize(img, resized_img, cv::Size(new_width, new_height));

		cv::imshow("view", resized_img);
		cv::waitKey(30);
	}
	catch (cv_bridge::Exception &e)
	{
		ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "image_listener");
	ros::NodeHandle nh;
	cv::namedWindow("view");
	cv::startWindowThread();
	image_transport::ImageTransport it(nh);
	image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
	ros::spin();
	cv::destroyWindow("view");
}
