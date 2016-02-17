#include <pluginlib/class_list_macros.h>
#include "alma_remote_planner.h"
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(alma::RemotePlanner, nav_core::BaseGlobalPlanner)


#define SERVER_TO_M 0.01
#define M_TO_SERVER_S 10000
#define M_TO_SERVER 100

using namespace std;
using namespace json11;

static Json pose2JsonArray(const geometry_msgs::PoseStamped &pose)
{
        return Json::array {round(M_TO_SERVER*pose.pose.position.x),
                            round(M_TO_SERVER*pose.pose.position.y),
                            round(tf::getYaw(pose.pose.orientation)*100)/100};
}

static string pose2String(const geometry_msgs::PoseStamped &pose)
{
  char s[100];
  sprintf(s,"[%.0f,%.0f,%.2f]",
          M_TO_SERVER*pose.pose.position.x,
          M_TO_SERVER*pose.pose.position.y,
          tf::getYaw(pose.pose.orientation));
  return string(s);
}

static geometry_msgs::PoseStamped jsonArray2pose(const Json &value)
{
        geometry_msgs::PoseStamped pose;
        auto values = value.array_items();
        pose.header.frame_id = "map";
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(value[2].number_value());
        pose.pose.position.x = value[0].number_value()*SERVER_TO_M;
        pose.pose.position.y = value[1].number_value()*SERVER_TO_M;
        return pose;
}

//Default Constructor
namespace alma {

geometry_msgs::PoseStamped RemotePlanner::poseInMapFrame(const geometry_msgs::PoseStamped &stamped_in)
{
        geometry_msgs::PoseStamped stamped_out;
        try{
                listener.transformPose("map", stamped_in, stamped_out);
                return stamped_out;
        }
        catch(tf::TransformException& ex) {
                ROS_ERROR("Received an exception trying to transform a pose: %s", ex.what());
                return stamped_out;
        }
}

RemotePlanner::RemotePlanner (){

}

RemotePlanner::RemotePlanner(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
        initialize(name, costmap_ros);
}

void RemotePlanner::initialize(std::string name, costmap_2d::Costmap2DROS* costmap_ros){
  if(!initialized_){
        ros::NodeHandle private_nh("~/" + name);
        private_nh.param("get_url", getPathUrl, cpr::Url{""});
        private_nh.param("post_url", postPathUrl, cpr::Url{""});
        private_nh.param("timeout", timeout, 1.0);
        private_nh.param("resolution", resolution_cm, 50);
        private_nh.param("method", method, string("get"));
        ROS_INFO("Intialized remote planner with GET %s, POST %s, method %s, with timeout %.1f s and resolution %d",
                 getPathUrl.data(),postPathUrl.data(),method.data(),timeout,resolution_cm);
        initialized_ = true;
  }
  else
  {
    ROS_WARN("This planner has already been initialized... doing nothing");
  }
}

string RemotePlanner::getPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end)
{
  string start_s = pose2String(start);
  string end_s = pose2String(end);
  string res = to_string(resolution_cm);
  auto params = cpr::Parameters{{"resolution", res},{"for","wheelchair"},
                                {"start",start_s},{"end",end_s},
                                {"geometry","false"}, {"poses","true"}};
  ROS_INFO("Send GET request for new path to %s with data %s",getPathUrl.data(),params.content.data());
  cpr::Response response = cpr::Get(getPathUrl, params,
                                    cpr::Header {{"Content-Type", "application/json"}},
                                    cpr::Timeout {(long)(1000 * timeout)});
  ROS_INFO("Got response %ld with content %s",response.status_code,response.text.data());
  if(response.status_code != 200)
  {
          ROS_INFO("Unvalid response, return that no path was found");
          return string("");
  }
  return response.text;
}

string RemotePlanner::postPlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& end)
{
  Json s = pose2JsonArray(start);
  Json e = pose2JsonArray(end);
  Json data = Json::object {{"start",s},{"end",e}};
  string res = to_string(resolution_cm);
  auto params = cpr::Parameters{{"resolution",res},{"geometry","false"}, {"poses","true"}};
  ROS_INFO("Send POST request for new path to %s with data %s and params %s",
           getPathUrl.data(), data.dump().data(), params.content.data());
  cpr::Response response = cpr::Post(postPathUrl, params, cpr::Body {{data.dump()}},
                                    cpr::Header {{"Content-Type", "application/json"}},
                                    cpr::Timeout {(long)(1000 * timeout)});
  ROS_INFO("Got response %ld with content %s",response.status_code,response.text.data());
  if(response.status_code != 201)
  {
          ROS_INFO("Unvalid response, return that no path was found");
          return string("");
  }
  return response.text;
}

bool RemotePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan ){

        ROS_INFO("Try to make a plan");
        geometry_msgs::PoseStamped s = poseInMapFrame(start);
        geometry_msgs::PoseStamped e = poseInMapFrame(goal);
        string response;
        if(method == "get")
        {
          response = getPlan(s,e);
        }
        else
        {
          response = postPlan(s,e);
        }
        if(response.empty())
        {
          return false;
        }
        string error;
        Json payload = Json::parse(response, error);
        if(!error.empty())
        {
                ROS_INFO("Unable to parse response payload as json: %s", error.data());
                return false;
        }
        if(!payload.has_shape({{"poses", json11::Json::ARRAY}}, error))
        {
                ROS_INFO("Payload has not the right shape: %s", error.data());
                return false;
        }
        for (const Json& value : payload["poses"].array_items())
        {
                plan.push_back(jsonArray2pose(value));
        }
        ROS_INFO("Successfully parsed payload. Added %lu waypoints to the path", plan.size());
        return true;
}

};
