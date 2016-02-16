#include <pluginlib/class_list_macros.h>
#include "alma_remote_planner.h"
#include <tf/transform_datatypes.h>

 //register this planner as a BaseGlobalPlanner plugin
 PLUGINLIB_EXPORT_CLASS(alma::RemotePlanner, nav_core::BaseGlobalPlanner)

 using namespace std;

 static string sendRequest(cpr::Url url, const char *data, float timeout_s)
 {
   long timeout_ms = long(timeout_s * 1000);
   auto r = cpr::Get(url, cpr::Body{data}, cpr::Header{{"Content-Type", "application/json"}});
   cout << r.status_code << endl;
   cout << r.text << endl;
   return r.text;
 }

static Json::Value pose2JsonArray(const geometry_msgs::PoseStamped &pose)
 {
    Json::Value value;
    double angle = tf::getYaw(pose.orientation);
    value.append(pose.position.x);
    value.append(pose.position.x);
    value.append(angle);
    return value;
}

static geometry_msgs jsonArray2pose(const Json::Value &value)
{
   geometry_msgs::PoseStamped pose;
   pose.pose.orientation = tf::createQuaternionMsgFromYaw(value[2])
   pose.pose.position.x = value[0];
   pose.pose.position.y = value[2];
   return pose;
}

 //Default Constructor
 namespace alma {

 Json::Value RemotePlanner::poseInMapFrame(const geometry_msgs::PoseStamped &stamped_in)
 {
    geometry_msgs::PoseStamped stamped_out;
    try{
       listener->transformPose("map", stamped_in, stamped_out);
       return pose2JsonArray(stamped_out);
    }
    catch(tf::TransformException& ex){
       ROS_ERROR("Received an exception trying to transform a pose: %s", ex.what());
       return Json::Value();
    }
 }

 RemotePlanner::RemotePlanner (){

 }

 RemotePlanner::RemotePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
   initialize(name, costmap_ros);
 }

 void RemotePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
    ros::NodeHandle private_nh("~/" + name);
    string getUrl;
    private_nh.param("get_url", getUrl, "");
    getPathUrl = cpr::Url{getUrl};
    private_nh.param("timeout", timeout, 1.0);
 }

 bool RemotePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

    Json::Value data;
    data["start"] = poseInMapFrame(start);
    data["end"] = poseInMapFrame(goal);
    string t = sendRequest(getPathUrl, data, timeout);
    Json::Value response;
    reader.parse(t, response);
    // Json::Value poses = response["poses"];
    for (const Json::Value& value : root["poses"])
    {
      plan.push_back(jsonArray2pose(value));
   }
   return true;
 };
