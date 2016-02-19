/* Copyright 2012 XYZ */
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
#include <string>
#include <vector>
#include "json11.hpp"


using std::string;
using geometry_msgs::PoseStamped;

#ifndef ALMA_REMOTE_PLANNER_SRC_ALMA_REMOTE_PLANNER_H_
#define ALMA_REMOTE_PLANNER_SRC_ALMA_REMOTE_PLANNER_H_

namespace alma {

class RemotePlanner : public nav_core::BaseGlobalPlanner {
 private:
    tf::TransformListener listener;
    int resolution_cm;
    cpr::Url getPathUrl, postPathUrl;
    PoseStamped poseInMapFrame(const PoseStamped &stamped_in);
    bool initialized_;
    string method;
    string getPlan(const PoseStamped& start, const PoseStamped& end);
    string postPlan(const PoseStamped& start, const PoseStamped& end);
    ros::Publisher plan_pub;
    bool publish_plan;
    cpr::Timeout timeout;
 public:
    RemotePlanner();
    RemotePlanner(string name, costmap_2d::Costmap2DROS* costmap_ros);

/** overridden classes from interface nav_core::BaseGlobalPlanner **/
void initialize(string name, costmap_2d::Costmap2DROS* costmap_ros);
bool makePlan(const PoseStamped& start, const PoseStamped& goal,
              std::vector<PoseStamped>& plan);
};
};  // namespace alma

#endif  // ALMA_REMOTE_PLANNER_SRC_ALMA_REMOTE_PLANNER_H_
