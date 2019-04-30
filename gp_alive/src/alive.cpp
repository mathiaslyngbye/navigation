#include "ros/ros.h"

#include <sstream>
#include <string>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alive");    
    ros::NodeHandle n;

    // Loop once per 5 sec
    ros::Rate loop_rate(0.2);
        

    while(ros::ok())
    {
        ROS_INFO("I am alive...");	
        ros::spinOnce();
        
	    // Sleep to match loop rate
	    loop_rate.sleep();
    }
}
