/* Copyright 2016 Jerome */

#include "./alma_remote_planner.h"
#include <pluginlib/class_list_macros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <string>
#include <vector>

// register this planner as a BaseGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(alma::RemotePlanner, nav_core::BaseGlobalPlanner)


#define SERVER_TO_M 0.01
#define M_TO_SERVER_S 10000
#define M_TO_SERVER 100

using std::string;
using json11::Json;

static cpr::Header header = cpr::Header {{"Content-Type", "application/json"}};

static Json pose2JsonArray(const PoseStamped &pose) {
    return Json::array {round(M_TO_SERVER*pose.pose.position.x),
                        round(M_TO_SERVER*pose.pose.position.y),
                        round(tf::getYaw(pose.pose.orientation)*100)/100};
}

// See remote_order_poller.py: target2Pose
/* Move base accept PoseStamped targets, i.e. we must specify an orientation.
When no orientation is provided, we pass yaw = 0 and rol, pitch != 0.
This way, the planners can check if a valid (2D) target angle is provided
and if not, they should issue a plan to travel towards a position,
ignoring the orientation (like the alma planner indeed does).
*/
static string pose2String(const PoseStamped &pose) {
    tf::Quaternion q;
    tf::quaternionMsgToTF(pose.pose.orientation, q);
    double yaw, pitch, roll;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);
    char s[100];
    ROS_INFO("RPY = [%.2f %.2f %.2f]", roll, pitch, yaw);
    if (yaw == 0 && roll !=0 && pitch != 0) {
        // Ignore orientation
        snprintf(s, sizeof(s), "[%.0f,%.0f]",
                 M_TO_SERVER*pose.pose.position.x,
                 M_TO_SERVER*pose.pose.position.y);
    } else {
        /// TODO: could use the computed yaw;
        snprintf(s, sizeof(s), "[%.0f,%.0f,%.2f]",
                M_TO_SERVER*pose.pose.position.x,
                M_TO_SERVER*pose.pose.position.y,
                tf::getYaw(pose.pose.orientation));
    }
    return string(s);
}

static PoseStamped jsonArray2pose(const Json &value) {
    PoseStamped pose;
    auto values = value.array_items();
    pose.header.frame_id = "map";
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(
        value[2].number_value());
    pose.pose.position.x = value[0].number_value()*SERVER_TO_M;
    pose.pose.position.y = value[1].number_value()*SERVER_TO_M;
    return pose;
}

// Default Constructor
namespace alma {

PoseStamped RemotePlanner::poseInMapFrame(const PoseStamped &stamped_in) {
    PoseStamped stamped_out;
        try {
            listener.transformPose("map", stamped_in, stamped_out);
            return stamped_out;
        }
        catch(tf::TransformException& ex) {
            ROS_ERROR("Received an exception trying to transform a pose: %s",
                      ex.what());
            return stamped_out;
        }
}

RemotePlanner::RemotePlanner() :  timeout{5000} {
}

RemotePlanner::RemotePlanner(string name,
                             costmap_2d::Costmap2DROS* costmap_ros) :
                             timeout{5000} {
    initialize(name, costmap_ros);
}

void RemotePlanner::initialize(string name,
                               costmap_2d::Costmap2DROS* costmap_ros) {
    if (!initialized_) {
        ros::NodeHandle private_nh("~/" + name);
        string server_uri;
        string world_id;
        int user_id;
        if (!ros::param::get("server_uri", server_uri)) {
            ROS_FATAL("Please provide parameter server_uri!");
            ros::shutdown();
        }
        if (!ros::param::get("world", world_id)) {
            ROS_FATAL("Please provide parameter world!");
            ros::shutdown();
        }
        if (!ros::param::get("user", user_id)) {
            ROS_FATAL("Please provide parameter user!");
            ros::shutdown();
        }
        char s[100];
        snprintf(s, sizeof(s), "%s/worlds/%s/map/path",
                 server_uri.data(), world_id.data());
        getPathUrl = cpr::Url {s};
        snprintf(s, sizeof(s), "%s/worlds/%s/users/%d/paths",
                 server_uri.data(), world_id.data(), user_id);
        postPathUrl = cpr::Url {s};
        double to;
        private_nh.param<double>("timeout", to, 5.0);
        long toi = round(1000 * to);
        timeout = cpr::Timeout {toi};
        private_nh.param("resolution", resolution_cm, 50);
        private_nh.param("method", method, string("get"));
        private_nh.param("publish_plan", publish_plan, false);
        if (publish_plan) {
            plan_pub = private_nh.advertise<nav_msgs::Path>("plan", 1);
        }
        ROS_INFO("Intialized remote planner with GET %s, POST %s, method %s, "
                 "with timeout %ld ms and resolution %d. Publish plan %d",
                 getPathUrl.data(), postPathUrl.data(), method.data(), toi,
                 resolution_cm, publish_plan);
        initialized_ = true;
    } else {
        ROS_WARN("This planner has already been initialized... doing nothing");
    }
}

string RemotePlanner::getPlan(const PoseStamped& start,
                              const PoseStamped& end) {
  string start_s = pose2String(start);
  string end_s = pose2String(end);
  string res = std::to_string(resolution_cm);
  auto params = cpr::Parameters{{"resolution", res}, {"for", "wheelchair"},
                                {"start", start_s}, {"end", end_s},
                                {"geometry", "false"}, {"poses", "true"}};
  ROS_INFO("Send GET request for new path to %s with data %s",
           getPathUrl.data(), params.content.data());
  cpr::Response response = cpr::Get(getPathUrl, params, header, timeout);
  ROS_INFO("Got response %ld with content %s",
           response.status_code,
           response.text.data());
  if (response.status_code != 200) {
          ROS_INFO("Unvalid response, return that no path was found");
          return string("");
  }
  return response.text;
}

string RemotePlanner::postPlan(const PoseStamped& start,
                               const PoseStamped& end) {
  Json s = pose2JsonArray(start);
  Json e = pose2JsonArray(end);
  Json data = Json::object {{"start", s}, {"end", e}};
  string res = std::to_string(resolution_cm);
  auto params = cpr::Parameters{{"resolution", res},
                                {"geometry", "false"},
                                {"poses", "true"}};
  auto body = cpr::Body {{data.dump()}};
  ROS_INFO("Send POST request for new path to %s with data %s and params %s",
           getPathUrl.data(), data.dump().data(), params.content.data());
  cpr::Response response = cpr::Post(postPathUrl, params, body, header,
                                     timeout);
  ROS_INFO("Got response %ld with content %s",
           response.status_code, response.text.data());
  if (response.status_code != 201) {
          ROS_INFO("Unvalid response, return that no path was found");
          return string("");
  }
  return response.text;
}

bool RemotePlanner::makePlan(const PoseStamped& start,
                             const PoseStamped& goal,
                             std::vector<PoseStamped>& plan ) {
        ROS_INFO("Try to make a plan");
        PoseStamped s = poseInMapFrame(start);
        PoseStamped e = poseInMapFrame(goal);
        string response;
        if (method == "get") {
          response = getPlan(s, e);
        } else {
          response = postPlan(s, e);
        }
        if (response.empty()) {
          return false;
        }
        string error;
        Json payload = Json::parse(response, error);
        if (!error.empty()) {
                ROS_INFO("Unable to parse response payload as json: %s",
                         error.data());
                return false;
        }
        if (!payload.has_shape({{"poses", json11::Json::ARRAY}}, error)) {
                ROS_INFO("Payload has not the right shape: %s", error.data());
                return false;
        }
        for (const Json& value : payload["poses"].array_items()) {
                plan.push_back(jsonArray2pose(value));
        }
        ROS_INFO("Successfully parsed payload. Added %lu waypoints to the path",
                 plan.size());
        if (publish_plan) {
          nav_msgs::Path msg = nav_msgs::Path();
          msg.header.frame_id = "map";
          msg.poses = plan;
          plan_pub.publish(msg);
        }
        return true;
}

};  // namespace alma
