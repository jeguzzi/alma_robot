#include <pluginlib/class_list_macros.h>
#include "alma_remote_planner.h"
#include <tf/transform_datatypes.h>

//register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(alma::RemotePlanner, nav_core::BaseGlobalPlanner)

using namespace std;
using namespace json11;

static string sendRequest(cpr::Url url, const char *data, float timeout_s)
{
        long timeout_ms = long(timeout_s * 1000);
        auto r = cpr::Get(url, cpr::Body {data}, cpr::Header {{"Content-Type", "application/json"}});
        cout << r.status_code << endl;
        cout << r.text << endl;
        return r.text;
}

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
}

bool RemotePlanner::makePlan(const geometry_msgs::PoseStamped& start, const geometry_msgs::PoseStamped& goal,  std::vector<geometry_msgs::PoseStamped>& plan ){

        Json data = Json::object {{"start",poseInMapFrame(start)},{"end",poseInMapFrame(goal)}};
        string t = sendRequest(getPathUrl, data.dump().data(), timeout);
        string error;
        Json response = Json::parse(t, error);
        for (const Json& value : response["poses"].array_items())
        {
                plan.push_back(jsonArray2pose(value));
        }
        return true;
}

};
