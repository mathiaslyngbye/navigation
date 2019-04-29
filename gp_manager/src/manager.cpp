#include "ros/ros.h"

#include <sstream>
#include <string>

std::string pod_name =      "pod_gp_navfn_node_local.yaml";
std::string pod_location =  "$(rospack find navigation-containers)/kubernetes/";
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "manager");

    ros::NodeHandle n;

    // Loop 10 times pr sec
    ros::Rate loop_rate(2);
        
    bool exists = false;
    std::string command = "";

    while(ros::ok())
    {
        // Args; service_name and print_failure_reason.
        exists = ros::service::exists("/global_planner/make_plan", true);	    

        if(!exists)
        {
            ROS_INFO("global_planner missing on ROS network. Spawning locally.");
            command = "kubectl apply -f " + pod_location + pod_name;
            system(command.c_str());
            ros::Duration(5).sleep();
            //system("roslaunch gp_launch gp_navfn_node.launch");
        }
        else 
        {
            ROS_INFO("global_planner exists on ROS network. Doing nothing...");
            //command = "kubectl delete -f " + pod_location + pod_name;
            //system(command.c_str());
        }
	
        ros::spinOnce();
        
	    // Sleep to match loop rate
	    loop_rate.sleep();

    }
}
