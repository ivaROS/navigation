#include <ros/ros.h>

#include <nav_core/base_global_planner.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>

#include <pluginlib/class_list_macros.h>

#include <fmm_global_planner/FMMPlan.h>

namespace fmm_global_planner
{

class FMMGlobalPlanner : public nav_core::BaseGlobalPlanner
{
public:
  FMMGlobalPlanner()
    : initialized_(false), costmap_ros_(nullptr), costmap_(nullptr)
  {
  }

  FMMGlobalPlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false), costmap_ros_(nullptr), costmap_(nullptr)
  {
    initialize(name, costmap_ros);
  }

  void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros) override
  {
    if (initialized_)
    {
      ROS_WARN("FMMGlobalPlanner already initialized");
      return;
    }

    ros::NodeHandle private_nh("~/" + name);

    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();

    private_nh.param<std::string>("fmm_service_name", fmm_service_name_, std::string("/fmm_make_plan"));
    private_nh.param<int>("occupied_threshold", occupied_threshold_, 50);

    fmm_client_ = private_nh.serviceClient<fmm_global_planner::FMMPlan>(fmm_service_name_);

    initialized_ = true;

    ROS_INFO("FMMGlobalPlanner initialized. Using service: %s", fmm_service_name_.c_str());
  }

  bool makePlan(const geometry_msgs::PoseStamped& start,
                const geometry_msgs::PoseStamped& goal,
                std::vector<geometry_msgs::PoseStamped>& plan) override
  {
    plan.clear();

    if (!initialized_)
    {
      ROS_ERROR("FMMGlobalPlanner has not been initialized");
      return false;
    }

    if (!costmap_)
    {
      ROS_ERROR("No costmap available");
      return false;
    }

    nav_msgs::OccupancyGrid map_msg;
    costmapToOccupancyGrid(map_msg);

    fmm_global_planner::FMMPlan srv;
    srv.request.start = start;
    srv.request.goal = goal;
    srv.request.map = map_msg;

    if (!fmm_client_.waitForExistence(ros::Duration(1.0)))
    {
      ROS_ERROR("FMM service not available: %s", fmm_service_name_.c_str());
      return false;
    }

    if (!fmm_client_.call(srv))
    {
      ROS_ERROR("Failed to call FMM service");
      return false;
    }

    if (!srv.response.success)
    {
      ROS_WARN("FMM planner failed: %s", srv.response.message.c_str());
      return false;
    }

    if (srv.response.path.poses.empty())
    {
      ROS_WARN("FMM returned empty path");
      return false;
    }

    plan = srv.response.path.poses;

    ROS_DEBUG("FMMGlobalPlanner returned path with %lu poses", plan.size());
    return true;
  }

private:
  bool initialized_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_;

  ros::ServiceClient fmm_client_;

  std::string fmm_service_name_;
  int occupied_threshold_;

  void costmapToOccupancyGrid(nav_msgs::OccupancyGrid& grid_msg)
  {
    const unsigned int size_x = costmap_->getSizeInCellsX();
    const unsigned int size_y = costmap_->getSizeInCellsY();
    const double resolution = costmap_->getResolution();
    const double origin_x = costmap_->getOriginX();
    const double origin_y = costmap_->getOriginY();

    grid_msg.header.stamp = ros::Time::now();
    grid_msg.header.frame_id = costmap_ros_->getGlobalFrameID();

    grid_msg.info.resolution = resolution;
    grid_msg.info.width = size_x;
    grid_msg.info.height = size_y;
    grid_msg.info.origin.position.x = origin_x;
    grid_msg.info.origin.position.y = origin_y;
    grid_msg.info.origin.position.z = 0.0;
    grid_msg.info.origin.orientation.w = 1.0;

    grid_msg.data.resize(size_x * size_y);

    for (unsigned int y = 0; y < size_y; ++y)
    {
      for (unsigned int x = 0; x < size_x; ++x)
      {
        unsigned char cost = costmap_->getCost(x, y);

        int8_t value = 0;

        if (cost == costmap_2d::NO_INFORMATION)
        {
          value = -1;
        }
        else if (cost == costmap_2d::LETHAL_OBSTACLE ||
                 cost == costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
                 cost >= occupied_threshold_)
        {
          value = 100;
        }
        else
        {
          value = 0;
        }

        grid_msg.data[y * size_x + x] = value;
      }
    }
  }
};

}  // namespace fmm_global_planner

PLUGINLIB_EXPORT_CLASS(fmm_global_planner::FMMGlobalPlanner, nav_core::BaseGlobalPlanner)