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

    // Create strong vector for holding "rosnode list"
    ros::V_string v;

    while(ros::ok())
    {
        // Reset exists flag
        exists = false;        

        // Clear previous list of nodes
        v.clear();   
        
        // get ros nodes on current master
        if(!ros::master::getNodes(v))
        {
            ROS_INFO("ROS master failed to get node list...");
        }
        else
        {
            for(int i = 0; i < v.size(); i++)
            {
                if(v[i] == "/alive")
                {
                    exists = true;
                    break;
                }    
            }
        }
        
        if(!exists)
        {
            ROS_INFO("Remote global_planner missing on ROS network.");
            if(!ros::service::exists("/global_planner/make_plan")
            {
                ROS_INFO("Local global_planner missing on ROS network.");
                ROS_INFO("Spawning local global_panner locally.");
                //command = "kubectl apply -f " + pod_location + pod_name;
                //system(command.c_str());
                ros::Duration(10).sleep();
            }
            else
            {
                ROS_INFO("Local global_planner exists on ROS network.");
                ROS_INFO("Doing nothing...");
            }
        }
        else 
        {
            ROS_INFO("Remote global_planner exists on ROS network.");
            ROS_INFO("Removing local global_planners...");
            //command = "kubectl delete -f " + pod_location + pod_name;
            //system(command.c_str());
        }
	
        ros::spinOnce();
        
	    // Sleep to match loop rate
	    loop_rate.sleep();

    }
}
