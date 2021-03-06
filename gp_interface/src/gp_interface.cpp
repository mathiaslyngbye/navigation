#include <pluginlib/class_list_macros.h>
#include <gp_interface/gp_interface.h>
#include <navfn/MakeNavPlan.h>
#include <navfn/SetCostmap.h>
#include <vector>
#include <string>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(gp_interface::GPInterface, nav_core::BaseGlobalPlanner)

using namespace std;

//Default Constructor
namespace gp_interface 
{

    GPInterface::GPInterface()
    {
        initialized_ = false;
    }

    GPInterface::GPInterface(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {
        initialized_ = false;
        initialize(name, costmap_ros);
    }

    void GPInterface::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    {      
        if(!initialized_)
        {
            // Initialize the costmap_ros_ attribute to the parameter.
            costmap_ros_ = costmap_ros;
            
            // Initialize the costmap_ from costmap_ros_
            costmap_ = costmap_ros_->getCostmap();
            
            // The following code is outcommented due to the set parameters service.
            // Service files exist, but the service was never implemented.
            // This resulted in some misconception.
            /*
            // Get costmap dimensions
            width_ = costmap_->getSizeInCellsX();
            height_ = costmap_->getSizeInCellsY();
            
            // Create cost array
            cells_ = width_*height_;
            
            // Initialize vector of costs
            std::vector<unsigned char, std::allocator<unsigned char> > costs_;
            
            
            // Fill costs[] with costs of costmap_
            {
                unsigned int mx = 0, my = 0;
                for(unsigned int i = 0; i < cells_; i++)
                {
                    if((i % width_) == 0)
                    {
                        my++;
                    }
                    mx = i % width_;
                    costs_.push_back(costmap_->getCost(mx,my));
                }
            }
            
            // Configure service client
            ros::NodeHandle nh;
            ros::ServiceClient client = nh.serviceClient<navfn::SetCostmap>("/global_planner/costmap/set_parameters");
            
            // Create service and set request variables
            navfn::SetCostmap srv;
            srv.request.costs = costs_;
            srv.request.height = height_;
            srv.request.width = width_;
            */
        }
        else
        {
            ROS_WARN("GPInterface has already been initialized... doing nothing");
        }
    }

    bool GPInterface::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal, std::vector<geometry_msgs::PoseStamped>& plan)
    {    
        // Create service client
        ros::NodeHandle n;
        
        ros::ServiceClient client = n.serviceClient<navfn::MakeNavPlan>("/global_planner/make_plan");
        
        // Create service and set service variables
        navfn::MakeNavPlan srv;
        srv.request.start = start;
        srv.request.goal = goal;
        
        // Create variables for containing responses
        unsigned int plan_found_ = false;
        std::string error_message_ = "";
        
        // Call service
        if(client.call(srv))
        {
            ROS_INFO("GPInterface succesfully received response navfn_node");    
        }
        else
        {
            ROS_ERROR("GPInterface failed to receive response from navfn_node");
            return false;
        }
        
        // Store service responses
        plan_found_ = srv.response.plan_found;
        error_message_ = srv.response.error_message; 
        
        // Return
        if(plan_found_)
        {
            ROS_INFO("GPInterface succesfully obtained a plan from navfn_node");   
            plan = srv.response.path;          
            return true;
        }
        else
        {
            ROS_WARN("GPInterface was unable to obtain a plan. The following error message was received from navfn_node: %s", error_message_.c_str());
            return false;
        }
    }
}
