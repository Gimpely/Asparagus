// Ros libraries
#include "ros/ros.h"
#include "std_msgs/Float32MultiArray.h"
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
bhf::ads::SetLocalAddress({192, 168, 65, 146, 1, 1});
AdsDevice route {remoteIpV4, remoteNetId, AMSPORT_R0_PLC_TC3};

// Read alphas from PLC (robot angle in degrees)
AdsVariable<std::array<float, 3>>alpha_Delta {route, "MAIN.DataExchange.PLC_to_PC.alpha_Delta"};


int main(int argc, char **argv)
{

    ros::init(argc, argv, "PLC_comm_pub");
    ros::NodeHandle nh;

    // Publish alphas (angles in degrees) from delta robot joints
    ros::Publisher alphas_pub = nh.advertise<std_msgs::Float32MultiArray>("alphas_pub", 1000);

    ros::Rate loop_rate(100);

    while (ros::ok())
    {   
   
        //std::cout <<" ADS read " << alpha_Delta_0 << '\n';

        // Publish data
        std_msgs::Float32MultiArray alphas;
        alphas.data.push_back(alpha_Delta_0);
        alphas.data.push_back(alpha_Delta_1);
        alphas.data.push_back(alpha_Delta_2);
        alphas.data.push_back(4.0);

        alphas_pub.publish(alphas);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}