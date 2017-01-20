/*
 * 用来订阅/sensor/imu_data与/sensor/camera1_image话题
 * imu姿态估计
 * 传输imu数据与图像数据给服务器
 */

#include "ros/ros.h"
#include "std_msgs/Header.h"
#include "std_msgs/Float32.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_imu/mpu6050.h"

#include "opencv2/highgui/highgui.hpp"

using namespace std;
using namespace cv;

void imuCallback(const sensor_imu::mpu6050 &msg)
{
	//std_msgs::Header  h = msg.header;
	double timeable = msg.header.stamp.sec + 1.0*msg.header.stamp.nsec/1000000000;
	boost::array<float, 3> accl = msg.accleration;
	boost::array<float, 3> angv  = msg.angular_velocity;
	boost::array<float, 3> ang = msg.angle;
	ROS_INFO("Timeable %lf secs.nsecs", timeable);
	ROS_INFO("Acceleration x %f y %f z %f", accl[0], accl[1], accl[2]);
	ROS_INFO("Angular velocity x %f y %f z %f", angv[0], angv[1], angv[2]);
	ROS_INFO("Angle x %f y %f z %f", ang[0], ang[1], ang[2]);
}

void cameraLeftCallback(const sensor_msgs::ImageConstPtr &msg)
{
	imshow("img", cv_bridge::toCvShare(msg, "bgr8")->image);
	waitKey(30);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "client_node");
	ROS_INFO("client begin ...");
	ros::NodeHandle n;
	ros::Subscriber imu_data_sub = n.subscribe("/serial_imu/imu_data", 10, imuCallback);
	// 为每个publisher提供多个可用的transport
	image_transport::ImageTransport it(n);
	image_transport::Subscriber camera_left_image_sub = it.subscribe("/camera_left/usb_cam/image_raw", 1, cameraLeftCallback);     
	ros::spin();
	return 0;
}