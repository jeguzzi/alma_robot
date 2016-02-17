/** include the libraries you need in your planner here */
/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <cpr/cpr.h>
#include "json11.hpp"

using std::string;

#ifndef ALMA_REMOTE_PLANNER_CPP
#define ALMA_REMOTE_PLANNER_CPP

namespace alma {

class RemotePlanner : public nav_core::BaseGlobalPlanner {

private:
        tf::TransformListener listener;
        double timeout;
        int resolution_cm;
        cpr::Url getPathUrl, postPathUrl;
        geometry_msgs::PoseStamped poseInMapFrame(const geometry_msgs::PoseStamped &stamped_in);
        bool initialized_;
        string method;
        std::string getPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end);
        std::string postPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end);
public:

        RemotePlanner();
        RemotePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped& start,
                      const geometry_msgs::PoseStamped& goal,
                      std::vector<geometry_msgs::PoseStamped>& plan
                      );
};
};
#endif
