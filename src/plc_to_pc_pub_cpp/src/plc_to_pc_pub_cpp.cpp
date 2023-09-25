// Ros libraries
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float32.h"

#include "beckhoff_msgs/array5.h"
//#include "std_msgs/MultiArrayDimension.h"
// Ads libraries
#include "../AdsLib/AdsLib.h"
#include "../AdsLib/AdsVariable.h"
#include "../AdsLib/AdsNotificationOOI.h"

#include <array>
#include <iostream>
#include <iomanip>


// Create route to the PLC
static const AmsNetId remoteNetId { 5, 59, 143, 52, 1, 1 };
static const char remoteIpV4[] = "192.168.65.174";
// uncomment and adjust if automatic AmsNetId deduction is not working as expected
//bhf::ads::SetLocalAddress({192, 168, 65, 146, 1, 1});
AdsDevice route {remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

// Define data to send to 
AdsVariable<std::array<double, 5>> omega_Delta {route, "MAIN.DataExchange.PC_to_PLC.omega_Delta"};


// Define publisher
ros::Publisher alpha_delta_pub; 

//create odometry to publish
/*nav_msgs::Odometry createOdom(const double robot_odom[]){

	static uint32_t  seq = 0;
	nav_msgs::Odometry odom;

	//header
	std_msgs::Header header;
	header.seq = seq;
	int64_t s = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
	int64_t ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - s * 1000000000;
	header.stamp.sec = s;
	header.stamp.nsec = ns;
	header.frame_id = "odom";
	odom.header = header;


	//frame_id
	odom.child_frame_id = "base_link";


	//pose
	geometry_msgs::PoseWithCovariance poseWC;
	geometry_msgs::Pose pose;

	geometry_msgs::Point position;
	position.x = robot_odom[0];
	position.y = robot_odom[1];
	position.z = 0;
	pose.position = position;

	geometry_msgs::Quaternion orientation;
	orientation.x = 0;
	orientation.y = 0;
	orientation.z = -1 * sin(robot_odom[2] / 2);
	orientation.w = cos(robot_odom[2] / 2);
	pose.orientation = orientation;
	poseWC.pose = pose;

	poseWC.covariance = { 0.003, 0, 0, 0, 0, 0,
		0, 0.003, 0, 0, 0, 0,
		0, 0, 0.003, 0, 0, 0,
		0, 0, 0, 0.003, 0, 0,
		0, 0, 0, 0, 0.003, 0,
		0, 0, 0, 0, 0, 0.003 };

	odom.pose = poseWC;


	//twist
	geometry_msgs::TwistWithCovariance twistWC;
	geometry_msgs::Twist twist;

	geometry_msgs::Vector3 linear;
	linear.x = robot_odom[3];
	linear.y = robot_odom[4];
	linear.z = 0;
	twist.linear = linear;

	geometry_msgs::Vector3 angular;
	angular.x = 0;
	angular.y = 0;
	angular.z = robot_odom[5];
	twist.angular = angular;

	twistWC.twist = twist;

	twistWC.covariance = { 0.01, 0, 0, 0, 0, 0,
		0, 0.01, 0, 0, 0, 0,
		0, 0, 0.01, 0, 0, 0,
		0, 0, 0, 0.01, 0, 0,
		0, 0, 0, 0, 0.01, 0,
		0, 0, 0, 0, 0, 0.01 };

	odom.twist = twistWC;
	if ((seq++)>4294967294){ seq = 0; }

	return odom;
}*/


//notification callback function - odometry create and publish on topic
/*void odometryCallback(const AmsAddr* pAddr, const AdsNotificationHeader* pNotification, uint32_t hUser){

	const double* data = reinterpret_cast<const double*>(pNotification + 1);
	nav_msgs::Odometry odom = createOdom(data);
	pub_odom.publish(odom);
}*/


// Notification callback function - read alphas from PLC and publish on topic
void alphasCallback(const AmsAddr* pAddr, const AdsNotificationHeader* pNotification, uint32_t hUser){

	const double* data = reinterpret_cast<const double*>(pNotification + 1);
    
    beckhoff_msgs::array5 alphas_delta;
    alphas_delta.data = {data[0],data[1],data[2],data[3],data[4]};

	alpha_delta_pub.publish(alphas_delta);
}

// Send omegas to PLC
void callback_receive_omegas(const beckhoff_msgs::array5& omega){
	//static float timeStamp = 0;
	//timeStamp += 0.05;
	omega_Delta = { omega.data[0], omega.data[1], omega.data[2], omega.data[3], omega.data[4] };
    //std::cout <<" ADS write " << omega.data[0] << '\n';
}


int main(int argc, char **argv)
{

    //std::cout <<" ADS read " << alpha_Delta_0 << '\n';

    ros::init(argc, argv, "ads_communication");
    ros::NodeHandle nh;

    //subscriber
	ros::Subscriber omega_Delta_sub = nh.subscribe("/omegas_delta", 1000, callback_receive_omegas);
	//ros::Subscriber sub_ESPG = nh.subscribe("/ESPG", 1000, callback_receive_ESPG);

    // Publish alphas (angles in degrees) from delta robot joints
    alpha_delta_pub = nh.advertise<beckhoff_msgs::array5 >("alpha_delta_pub", 1000);
    
    //notifications
	const AdsNotificationAttrib attrib = { sizeof(double)* 5, ADSTRANS_SERVERCYCLE, 0, { 10000 } };
	AdsNotification notification{ route, "MAIN.DataExchange.PLC_to_PC.alpha_Delta", attrib, &alphasCallback, 0xBEEFDEAD };
    

	ros::spin();
	return 0;
}