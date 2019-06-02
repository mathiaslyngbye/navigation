#include "ros/ros.h"

#include <sstream>
#include <string>

std::string pod_name =      "rosnode-gp";
std::string pod_file =      "pod_gp_navfn_node_local.yaml";
std::string pod_location =  "$(rospack find navigation-containers)/kubernetes/";
 
int main(int argc, char **argv)
{
    // Create ros node
    ros::init(argc, argv, "manager");
    ros::NodeHandle n;

    // Loop 10 times pr sec
    ros::Rate loop_rate(2);
    
    // Create node search flags
    bool alive_exists = false;
    bool alive_exists_prev = false;
    bool gp_exists = false;
    std::string command = "";

    // Create strong vector for holding "rosnode list"
    ros::V_string v;

    while(ros::ok())
    {
        // Reset exists flag
        alive_exists_prev = alive_exists;
        alive_exists = false;
        gp_exists = false;        

        // Clear previous list of nodes
        v.clear();   
        
        // Get ros nodes on current master
        if(!ros::master::getNodes(v))
        {
            ROS_INFO("ROS master failed to get node list...");
        }
        else
        {
            for(int i = 0; i < v.size(); i++)
            {   
                // Check for existence of alive or global_planner node
                if(v[i] == "/alive")
                {
                    alive_exists = true;
                }
                else if(v[i] == "/global_planner")
                {
                    gp_exists = true;
                }    
            }
        }
        
        if(alive_exists && gp_exists)
        {   
            ROS_INFO("Remote global_planner exists on ROS network.");
            
            if(!alive_exists_prev)
            {
                // Output
                ROS_INFO("Removing local global_planners...");
                
                // Dirty manual regex system call   
                std::string cmd1 =  "kubectl exec -it $(kubectl get pods -o=name | grep " + pod_name + 
                                    " | sed \"s/^.\\{4\\}//\") -- bash -c \"source root/catkin_ws/devel/setup.bash && rosnode kill global_planner\"";
                std::string cmd2 =  "kubectl delete -f " + pod_location + pod_file;
                command = cmd1 + " && " + cmd2;
                system(command.c_str());
            }
            else
            {
                ROS_DEBUG("Doing nothing...");
            }
        }
        else if(!alive_exists && gp_exists)
        {
            // Output
            ROS_INFO("Remote global_planner missing on ROS network.");
            ROS_INFO("Local global_planner exists on ROS network.");
            ROS_DEBUG("Doing nothing...");
        }
        else if(!alive_exists && !gp_exists)
        {   
            // Output
            ROS_INFO("Remote global_planner missing on ROS network.");
            ROS_INFO("Local global_planner missing on ROS network.");
            ROS_INFO("Spawning global_panner locally...");

            // Spawn local global_planner;
            command = "kubectl apply -f " + pod_location + pod_file;
            system(command.c_str());
            ros::Duration(10).sleep();
        }
        /*
        else if(alive_exists && !gp_exists)
        {
            // Error state
            ROS_WARN("Error on remote host.")
            ROS_INFO("Spawning global_planner locally...")
            
            // Spawn local global_planner;
            command = "kubectl apply -f " + pod_location + pod_file;
            system(command.c_str());
            ros::Duration(10).sleep();
        }
	    */

        //ROS_INFO("You reached the end of the loop! Congrats!"); 
        ros::spinOnce();
        
	    // Sleep to match loop rate
	    loop_rate.sleep();

    }
}
