#include <raspicam/raspicam.h>
#include <raspicam/raspicam_cv.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <std_msgs/Header.h>
#include <camera_info_manager/camera_info_manager.h>

using namespace raspicam;


int main(int argc, char **argv) {
    ros::init(argc, argv, "rosberrypi_cam");
    ros::NodeHandle nh("~");
    RaspiCam_Cv camera_cv;
    camera_cv.set(CV_CAP_PROP_FORMAT, CV_8UC1);
    if(!camera_cv.open())
        ROS_ERROR("Error opening camera");
    sleep(3);
    camera_cv.grab();
	cv::Mat cv_img;
	
	std::string camera_info_url;
	nh.param<std::string>("camera_info_url", camera_info_url, "");
    image_transport::ImageTransport it(nh);
    image_transport::CameraPublisher pub = it.advertiseCamera("image_raw", 1);
	std::string camera_name = nh.getNamespace();
	camera_info_manager::CameraInfoManager cinfo_(nh, camera_name);

	int timeout = camera_cv.get(CV_CAP_PROP_EXPOSURE);
	std::cout << 1./timeout << std::endl;
	ros::Rate rate(10);
    while(ros::ok()) {
		camera_cv.grab();
		camera_cv.retrieve(cv_img);
        std_msgs::Header header();
        cv_bridge::CvImage imgmsg;
		sensor_msgs::CameraInfo ci = cinfo_.getCameraInfo();
        imgmsg.header.frame_id = camera_name + "_optical_frame";
		ci.header.frame_id = imgmsg.header.frame_id;
        imgmsg.encoding = "mono8";
        imgmsg.image = cv_img;
        pub.publish(*imgmsg.toImageMsg(), ci, ros::Time::now());
        rate.sleep();
        ros::spinOnce();
    }
    return 0;
}
