// Ros libraries
#include "ros/ros.h"
// Include msg to communicate with tracks
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Pose2D.h"
// Include msg to communicate with robot
#include "beckhoff_msgs/CmdRobot.h"
#include "beckhoff_msgs/JointStateRobot.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "rospy_tutorials/Floats.h"
// Raw message for sending to PLC
#include "beckhoff_msgs/dataArray.h"
// Ads libraries
#include "../AdsLib/AdsLib.h"
#include "../AdsLib/AdsVariable.h"
#include "../AdsLib/AdsNotificationOOI.h"

#include <array>
#include <iostream>
#include <iomanip>
#include <math.h>
#include <chrono>


// Create route to the PLC
static const AmsNetId remoteNetId { 5, 59, 143, 52, 1, 1 };
static const char remoteIpV4[] = "192.168.1.220";
// uncomment and adjust if automatic AmsNetId deduction is not working as expected
//bhf::ads::SetLocalAddress({192, 168, 65, 146, 1, 1});
AdsDevice route {remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

// Define data to send to delta robot
AdsVariable<std::array<float, 20>> delta_to_plc {route, "MAIN.RobotDataExchange.PC_to_PLC.dataArray"};
// Define data to send to delta robot
AdsVariable<std::array<float, 20>> tracks_to_plc {route, "Odometry.CaterpillarDataExchange.PC_to_PLC.dataArray"};

// Define delta robot publisher
ros::Publisher delta_pub; 
// Publish tracks velocity in local cs frame
ros::Publisher tracks_vel_pub; 
// Publish tracks pose2D in global cs frame
ros::Publisher tracks_pose_pub; 

int16_t counter;
auto start = std::chrono::steady_clock::now();
auto end = std::chrono::steady_clock::now();

float home_gripper = false;
float reset_odom = false;


// Notification callback function - read alphas from PLC and publish on topic
void callback_delta_from_plc(const AmsAddr* pAddr, const AdsNotificationHeader* pNotification, uint32_t hUser){
	//counter = counter + 1;

	if (counter==0)
	{
		start = std::chrono::steady_clock::now();	
		counter = 1;
	}
	
	// static auto last_time = std::chrono::high_resolution_clock::now();  // static ensures this is kept between calls

    // auto current_time = std::chrono::high_resolution_clock::now();
    // std::chrono::duration<double, std::milli> elapsed = current_time - last_time;

    // std::cout << "Time since last call: " << elapsed.count() << " ms" << std::endl;

    // last_time = current_time;



	//if (counter==1){
	//start = std::chrono::steady_clock::now();
	//}
	//else{
	//	end = std::chrono::steady_clock::now();

	//	std::cout << "Elapsed time in milliseconds: "
    //    << std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
    //    << " µs" << std::endl;

	//	counter = 0;
	//}
	
	const float* data = reinterpret_cast<const float*>(pNotification + 1);
    
	// Read raw data from PLC
    beckhoff_msgs::dataArray delta_receive;
	// Timestamp, 5 position from motor encoders
    delta_receive.data = { data[0], data[1], data[2], data[3], data[4],
						data[5], data[6], data[7], data[8], data[9],
						data[10], data[11], data[12], data[13], data[14],
						data[15], data[16], data[17], data[18], data[19] };

	// Read timestamp
	beckhoff_msgs::JointStateRobot RobotJointState;
	// Read actual positions

	//int32_t now_sec = int(floor(delta_receive.data[0]*0.001));
	//int32_t now_nsec = int(delta_receive.data[0]*0.001*1e9)% int(1e9) ;
	//RobotJointState.Timestamp.sec = now_sec;
	//RobotJointState.Timestamp.nsec = now_nsec;
	RobotJointState.Timestamp.sec  = int(delta_receive.data[0]);
	RobotJointState.Timestamp.nsec = int((delta_receive.data[0] - int(delta_receive.data[0]))*1e9);

	// static auto oldStamp = RobotJointState.Timestamp.sec;
	// auto currStamp = RobotJointState.Timestamp.sec;
	// double diff = currStamp - oldStamp;
	// std::cout << "Stamp since last call: " << diff << std::endl;
	// oldStamp = currStamp;

	RobotJointState.qq.j0 = delta_receive.data[1];
	RobotJointState.qq.j1 = delta_receive.data[2];
	RobotJointState.qq.j2 = delta_receive.data[3];
	RobotJointState.qq.j3 = delta_receive.data[4];
	RobotJointState.qq.j4 = delta_receive.data[5];

	// std::cout << "Podatki: " << RobotJointState.Timestamp.sec << " " <<RobotJointState.qq.j3 << "\n";
	// std::cout << RobotJointState.qq.j4 << " " << "\n";


	// ROS_INFO("comm tstamp: %f", delta_receive.data[0]);
	// ROS_INFO("comm q 1: %f", delta_receive.data[1]);
	// ROS_INFO("comm q 2: %f", delta_receive.data[2]);
	// ROS_INFO("comm q 3: %f", delta_receive.data[3]);

	// Read actual velocities
	RobotJointState.dq.j0 = delta_receive.data[6];
	RobotJointState.dq.j1 = delta_receive.data[7];
	RobotJointState.dq.j2 = delta_receive.data[8];
	RobotJointState.dq.j3 = delta_receive.data[9];
	RobotJointState.dq.j4 = delta_receive.data[10];

	delta_pub.publish(RobotJointState);

}

// Send omegas to PLC
void callback_delta_to_plc(const beckhoff_msgs::CmdRobot& data){

	// Timestamp, omegas to PLC
	float now_time = float(data.Timestamp.sec) + float(data.Timestamp.nsec) / 10e-9;
	delta_to_plc = {now_time, data.dq.j0, data.dq.j1, data.dq.j2, data.dq.j3, data.dq.j4, home_gripper, float(data.open_gripper) , float(data.close_gripper),0,0,0,0,0,0,0,0,0,0,0};
	//delta_to_plc = { 0, 0.0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };
	// std::cout << "---------j4-------" << data.dq.j4 << '\n';
	//std::cout <<" ADS write " << now_time << '\n';
	// std::cout <<" Open gripper " << data << '\n';
	
	if (counter==1)
	{
		end = std::chrono::steady_clock::now();

		//std::cout << "Elapsed time in microseconds: "
		//<< std::chrono::duration_cast<std::chrono::microseconds>(end - start).count()
		//<< " " << std::endl;

		counter = 0;
	}
}	

void callback_zero_gripper(const std_msgs::Bool& data){
	home_gripper = float(data.data);
}


// Notification callback function - read odometry data
void callback_cat_from_plc(const AmsAddr* pAddr, const AdsNotificationHeader* pNotification, uint32_t hUser){

	const float* data = reinterpret_cast<const float*>(pNotification + 1);
    
	// Read raw data from PLC
    beckhoff_msgs::dataArray tracks_receive;
	// 
    tracks_receive.data = { data[0], data[1], data[2], data[3], data[4],
						data[5], data[6], data[7], data[8], data[9],
						data[10], data[11], data[12], data[13], data[14],
						data[15], data[16], data[17], data[18], data[19] };

	
	// Read velocity of the caterpillar robot in local frame
	geometry_msgs::TwistStamped Cat_velocity;
	
	Cat_velocity.header.stamp.sec  = int(tracks_receive.data[0]);
	Cat_velocity.header.stamp.nsec = int((tracks_receive.data[0] - int(tracks_receive.data[0]))*1e9);
	Cat_velocity.twist.linear.x  = tracks_receive.data[1];
	Cat_velocity.twist.angular.z = tracks_receive.data[2];

	tracks_vel_pub.publish(Cat_velocity);
	
	// Read pose of the caterpillar robot
	geometry_msgs::Pose2D Cat_pose2D;

	Cat_pose2D.x     = tracks_receive.data[3];
	Cat_pose2D.y     = tracks_receive.data[4];
	Cat_pose2D.theta = tracks_receive.data[5];

	tracks_pose_pub.publish(Cat_pose2D);

	//std::cout <<" ADS read X " << Cat_pose2D.x << '\n';
	//std::cout <<" ADS read Y " << Cat_pose2D.y << '\n';
	//std::cout <<" ADS read Theta " << Cat_pose2D.theta << '\n';
}

// Send data to caterpillars
void callback_cat_to_plc(const geometry_msgs::TwistStamped& data){
	
	// Send linear and angular velocities, timestamp to PLC
	// Timestamp, omegas to PLC
	float now_time = float(data.header.stamp.sec) + float(data.header.stamp.nsec) / 10e-9;
	tracks_to_plc = {now_time, float(data.twist.linear.x), float(data.twist.angular.z), reset_odom, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//delta_to_plc = { 0, 0.0, 0, 0, 0, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0 };

	//std::cout <<" ADS write " << reset_odom << '\n';
    
}

void callback_reset_odom(const std_msgs::Bool& data){
	reset_odom = float(data.data);
}


int main(int argc, char **argv)
{

    //std::cout <<" ADS read " << alpha_Delta_0 << '\n';

    ros::init(argc, argv, "ads_communication");
    ros::NodeHandle nh;

	////////////    Publish and subscribe to robot control    ///////////////////

    //subscriber
	ros::Subscriber omegas_sub = nh.subscribe("/robot/cmd", 1, callback_delta_to_plc);
	// Subscribe to cmd for gripper zeroing
	ros::Subscriber zero_sub = nh.subscribe("/robot/zero", 1, callback_zero_gripper);
	
    // Publish alphas (angles in degrees) from delta robot joints
    delta_pub = nh.advertise<beckhoff_msgs::JointStateRobot >("/robot/joint_state", 1);
    
    //notifications -> delta robot
	const AdsNotificationAttrib attrib = { sizeof(float)* 20, ADSTRANS_SERVERCYCLE, 0, { 10000 } };
	AdsNotification notification{ route, "MAIN.RobotDataExchange.PLC_to_PC.dataArray", attrib, &callback_delta_from_plc, 0xBEEFDEAD };


	////////////    Publish and subscribe to tracks control    ///////////////////
    
	//subscriber
	ros::Subscriber tracks_sub = nh.subscribe("/tracks/twist_cmd", 1, callback_cat_to_plc);
	ros::Subscriber odometry_reset_sub = nh.subscribe("/tracks/reset_odom", 1, callback_reset_odom);
	
    // Publish velocity and global position of tracks robot
    tracks_vel_pub  = nh.advertise<geometry_msgs::TwistStamped >("/tracks/twist_state", 1);
	tracks_pose_pub = nh.advertise<geometry_msgs::Pose2D >("/tracks/pose", 1);
    
    //notifications -> tracks robot
	const AdsNotificationAttrib attrib_tracks = { sizeof(float)* 20, ADSTRANS_SERVERCYCLE, 0, { 10000 } };
	AdsNotification notification_tracks{ route, "Odometry.CaterpillarDataExchange.PLC_to_PC.dataArray", attrib_tracks, &callback_cat_from_plc, 0xBEEFDEAD };


	//notifications -> odometry (caterpillars)
	//const AdsNotificationAttrib attrib1 = { sizeof(double)* 20, ADSTRANS_SERVERCYCLE, 0, { 10000 } };
	//AdsNotification notification1{ route, "Odometry.CaterpillarDataExchange.PLC_to_PC.dataArray", attrib1, &callback_cat_from_plc, 0xBEEFDEAD };
    

	ros::spin();
	return 0;
}