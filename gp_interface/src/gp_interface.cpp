#include <pluginlib/class_list_macros.h>
#include <gp_interface/gp_interface.h>
#include <navfn/MakeNavPlan.h>
#include <navfn/SetCostmap.h>
#include <vector>

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
        /*  SetCostmap.srv
        
            uint8[] costs
            uint16 height
            uint16 width
            ---
        */
        
        // Initialize the costmap_ros_ attribute to the parameter.
        costmap_ros_ = costmap_ros;
        
        // Initialize the costmap_ from costmap_ros_
        costmap_ = costmap_ros_->getCostmap();
        
        // Get costmap dimensions
        width_ = costmap_->getSizeInCellsX();
        height_ = costmap_->getSizeInCellsY();
        
        // Create cost array
        cells_ = width_*height_;
        
        //unsigned int costs_[cells_];                                          // Attempt no. 1
        //std::vector<unsigned int> costs_;                                     // Attempt no. 2
        std::vector<unsigned char, std::allocator<unsigned char> > costs_;      // Attempt no. 3
        
        
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
                costs_[i] = costmap_->getCost(mx,my);
                costs_.push_back(costmap_->getCost(mx,my));
            }
        }
        
        // Set costmap service (hopefully)
        ros::NodeHandle n;
        ros::ServiceClient client = n.serviceClient<navfn::SetCostmap>("SetCostmap");
        
        navfn::SetCostmap srv;
        
        srv.request.costs = costs_;
        srv.request.height = height_;
        srv.request.width = width_;

        
        if(client.call(srv))
        {
            ROS_INFO("GPInterface succesfully performed SetCostmap for navfn!");
        }
        else
        {
            ROS_ERROR("GPInterface failed to perform SetCostmap for navfn!");
        }   
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
