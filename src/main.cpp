#include "radar/radar.h"

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <can_msgs/Frame.h>
#include <std_msgs/Float32.h>

#include <string>

const int kNodeLoopRate = 500; 
const std::string kNodeName  = "radar"; //node name

int main(int argc, char **argv)
{

	ros::init(argc, argv, kNodeName);

	ros::NodeHandle nh(kNodeName);
	ros::Rate loop_rate(kNodeLoopRate);
	
	Radar radar(&nh);

	while (ros::ok())
	{
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
