#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream> // for converting the command line parameter to integer,用于将命令行参数转换为整数
#include <boost/array.hpp>

int main(int argc, char **argv)
{
    // Check if video source has been passed as a parameter,检查视频源是否已作为参数传递
    if (argv[1] == NULL)
    {
        ROS_INFO("argv[1]=NULL\n");
        return 1;
    }

    ros::init(argc, argv, "image_publisher"); // Initialize node,初始化节点
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1); // Publish topic，发布话题

    // added camera info topic
    ros::Publisher cam_info_pub = nh.advertise<sensor_msgs::CameraInfo>("camera/camera_info", 1);

    ros::Rate loop_rate(200); // refresh Hz.

    // Convert the passed as command line parameter index for the video device to an integer，
    // 将作为命令行参数传递的视频设备索引转换为整数
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    // Check if it is indeed a number，检查它是否确实是一个数字
    if (!(video_sourceCmd >> video_source))
    {
        ROS_INFO("video_sourceCmd is %d\n", video_source);
        return 1;
    }

    cv::VideoCapture cap(video_source);
    // Check if video device can be opened with the given index，检查是否可以使用给定的索引打开视频设备
    if (!cap.isOpened())
    {
        ROS_INFO("can not opencv video device\n");
        return 1;
    }
    cv::Mat frame;
    sensor_msgs::ImagePtr msg_ptr;
    sensor_msgs::CameraInfo info_msg;

    // assemble the camera info
    // known that camera height 1944, width 2592. Assume there is no distortion.
    info_msg.header.frame_id = "joint6_flange";
    info_msg.height = 1944;
    info_msg.width = 2592;
    info_msg.distortion_model = "plumb_bob";
    std::vector<double> D{0, 0, 0, 0, 0, 0};
    boost::array<double, 9> K = {
        2592, 0, 1296,
        0, 2592, 972,
        0, 0, 1,
    };
    boost::array<double, 12> P = {
        2592, 0, 1296, 0,
        0, 2592, 972, 0,
        0, 0, 1, 0,
    };
    info_msg.D = D;
    info_msg.K = K;
    info_msg.P = P;

    while (nh.ok())
    {
        cap >> frame;
        // cv::imshow("veiwer", frame);
        // Check if grabbed frame is actually full with some content，检查抓取的帧是否实际上充满了一些内容
        if (!frame.empty())
        {
            msg_ptr = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            msg_ptr->header.frame_id = "joint6_flange";
            pub.publish(msg_ptr);



            cam_info_pub.publish(info_msg);
            //cv::Wait(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
        // if(cv::waitKey(2) >= 0)
        // break;
    }
}
