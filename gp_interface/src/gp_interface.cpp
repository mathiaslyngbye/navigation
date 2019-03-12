#include <pluginlib/class_list_macros.h>
#include <gp_interface/gp_interface.h>
#include <navfn/MakeNavPlan.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(gp_interface::GPInterface, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace gp_interface {

    GPInterface::GPInterface()
    {

    }

    GPInterface::GPInterface(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialize(name, costmap_ros);
    }

    void GPInterface::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {

    }

    bool GPInterface::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<navfn::MakeNavPlan>("MakeNavPlan");
        
        navfn::MakeNavPlan srv;
        
        srv.request.start = start;
        srv.request.goal = goal;
        
        if(client.call(srv))
        {
            ROS_INFO("GPInterface succesfully obtained plan from navfn!");
            plan = srv.response.path;
        }
        else
        {
            ROS_ERROR("GPInterface failed to obtain plan from navfn!");
        }
        
        return true;
    }
};
