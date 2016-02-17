#include <pluginlib/class_list_macros.h>
#include "alma_remote_planner.h"
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(alma::RemotePlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace json11;

static Json pose2JsonArray(const geometry_msgs::PoseStamped &pose)
{
        return Json::array {pose.pose.position.x, pose.pose.position.y, tf::getYaw(pose.pose.orientation)};
}

static geometry_msgs::PoseStamped jsonArray2pose(const Json &value)
{
        geometry_msgs::PoseStamped pose;
        auto values = value.array_items();
        pose.header.frame_id = "map";
        pose.pose.orientation = tf::createQuaternionMsgFromYaw(value[2].number_value());
        pose.pose.position.x = value[0].number_value();
        pose.pose.position.y = value[1].number_value();
        return pose;
}

//Default Constructor
namespace alma {

Json RemotePlanner::poseInMapFrame(const geometry_msgs::PoseStamped &stamped_in)
{
        geometry_msgs::PoseStamped stamped_out;
        try{
                listener->transformPose("map", stamped_in, stamped_out);
                return pose2JsonArray(stamped_out);
        }
        catch(tf::TransformException& ex) {
                ROS_ERROR("Received an exception trying to transform a pose: %s", ex.what());
                return Json::object {};
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
        private_nh.param("get_url", getUrl, string(""));
        getPathUrl = cpr::Url {getUrl};
        private_nh.param("timeout", timeout, 1.0);
        ROS_INFO("Intialized remote planner at url %s with timeout %.1f s",getUrl.data(),timeout);
}

bool RemotePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,
                             std::vector<geometry_msgs::PoseStamped>& plan ){
        Json data = Json::object {{"start", poseInMapFrame(start)},{"end", poseInMapFrame(goal)}};
        ROS_INFO("Send request for new path to %s with data %s",getPathUrl.data(),data.dump().data());
        cpr::Response response = cpr::Get(getPathUrl, cpr::Body {data.dump().data()},
                                          cpr::Header {{"Content-Type", "application/json"}},
                                          cpr::Timeout{(long)(1000 * timeout)});
        ROS_INFO("Got response %d with content %s",response.status_code,response.text.data());
        if(response.status_code != 200)
        {
          ROS_INFO("Unvalid response, return that no path was found");
          return false;
        }

        string error;
        Json payload = Json::parse(response.text, error);
        if(error.empty())
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
        ROS_INFO("Succesufully parsed payload. Added %lu waypoints to the path", plan.size());
        return true;
}

};
