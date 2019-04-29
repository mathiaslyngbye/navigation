#include "ros/ros.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager");

    ros::NodeHandle n;

    // Set loop rate 10 times pr sec
    ros::Rate loop_rate(10);
    
    bool exists = false;

    while(ros::ok())
    {
        // Args; service_name and print_failure_reason.
        bool exists = ros::service::exists("/global_planner/make_plan", true);
        ROS_INFO("global_planner exists on ROS network. Doing nothing...");	    

        if(!exists)
        {
            system("roslaunch gp_launch gp_navfn_node.launch");
        }
	
        ros::spinOnce();
        
	// Sleep appropriate time to match loop rate
	loop_rate.sleep();

    }
}
