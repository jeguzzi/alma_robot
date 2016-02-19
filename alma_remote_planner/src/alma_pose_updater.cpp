/* Copyright 2016 Jerome */

#include <ros/ros.h>
#include <cpr/cpr.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <string>
#include "json11.hpp"


using json11::Json;
using geometry_msgs::PoseWithCovarianceStamped;
using std::string;

static cpr::Url pose_put_url;
static cpr::Url pose_get_url;
static ros::Publisher pose_pub;
static ros::Time last_ts;
static double period = 1;
static cpr::Timeout timeout{5};
static cpr::Header header = cpr::Header {{"Content-Type", "application/json"}};

#define SERVER_TO_M 0.01
#define M_TO_SERVER_S 10000
#define M_TO_SERVER 100

static Json pose2json(const PoseWithCovarianceStamped &msg) {
    auto cov =  msg.pose.covariance;
    Json jcov =  Json::array {
        cov[0]*M_TO_SERVER_S, cov[1]*M_TO_SERVER_S, cov[5]*M_TO_SERVER_S,
        cov[6]*M_TO_SERVER_S, cov[7]*M_TO_SERVER_S, cov[11]*M_TO_SERVER_S,
        cov[30]*M_TO_SERVER, cov[31]*M_TO_SERVER, cov[35]
    };
    Json pose2D = Json::object {
        {"pose2D", Json::object {
                {"x", msg.pose.pose.position.x*M_TO_SERVER},
                {"y", msg.pose.pose.position.y*M_TO_SERVER},
                {"theta",  tf::getYaw(msg.pose.pose.orientation)}}},
        {"covariance", jcov}};
    return Json::object {{"ego_pose", pose2D}};
}

static PoseWithCovarianceStamped json2pose(const Json &value) {
    PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    /// TODO: check if value has field 'covariance'
    string err;
    if (value.has_shape({{"covariance", Json::ARRAY}}, err)) {
        auto cov = value["covariance"].array_items();
        if (cov.size() == 9) {
            msg.pose.covariance[0] = cov[0].number_value();
            msg.pose.covariance[1] = cov[1].number_value();
            msg.pose.covariance[5] = cov[2].number_value();
            msg.pose.covariance[6] = cov[3].number_value();
            msg.pose.covariance[7] = cov[4].number_value();
            msg.pose.covariance[11] = cov[5].number_value();
            msg.pose.covariance[30] = cov[6].number_value();
            msg.pose.covariance[31] = cov[7].number_value();
            msg.pose.covariance[35] = cov[8].number_value();
        }
    }
    if (value.has_shape({{"pose2D", Json::OBJECT}}, err)) {
        Json pose_value = value["pose2D"];
        Json::shape pose_shape = Json::shape {
            {"x", Json::NUMBER},
            {"y", Json::NUMBER},
            {"theta", Json::NUMBER}};
        auto pose = msg.pose.pose;
        if (pose_value.has_shape(pose_shape, err)) {
            pose.position.x = pose_value["x"].number_value()*SERVER_TO_M;
            pose.position.y = pose_value["y"].number_value()*SERVER_TO_M;
            auto q  = tf::createQuaternionMsgFromYaw(
                pose_value["theta"].number_value());
            pose.orientation =  q;
        }
    }
    return msg;
}

static void send_update_pose_request(const PoseWithCovarianceStamped &msg) {
    double delta_time = (msg.header.stamp - last_ts).toSec();
    if (delta_time > period) {
        last_ts = msg.header.stamp;
        Json data = pose2json(msg);
        auto r = cpr::Put(pose_put_url, cpr::Body {data.dump().data()}, header);
        ROS_INFO("Pose updated: %ld", r.status_code);
    }
}

static void publish_initial_pose() {
    cpr::Response r = cpr::Get(pose_get_url, header, timeout);
    ROS_INFO("Fetched current pose. Got response %ld with content %s",
             r.status_code, r.text.data());
    if (r.status_code == 200) {
        string error;
        Json payload = Json::parse(r.text, error);
        if (!error.empty()) {
            ROS_INFO("Unable to parse response payload as json: %s",
                     error.data());
            return;
        }
        if (!payload.has_shape({{"pose", Json::OBJECT}}, error)) {
            ROS_INFO("Payload has not the right shape: %s", error.data());
            return;
        }
        Json value = payload["pose"];
        PoseWithCovarianceStamped pose = json2pose(value);
        pose_pub.publish(pose);
        ROS_INFO("Could parse the payload and publish a pose");
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "remote_pose_updater");
    ros::NodeHandle pnh("~");
    ros::NodeHandle n;
    last_ts = ros::Time(0);
    string server_uri;
    string world_id;
    int wheelchair_id;
    if (!ros::param::get("server_uri", server_uri)) {
        ROS_FATAL("Please provide parameter server_uri!");
        return 1;
    }
    if (!ros::param::get("world", world_id)) {
        ROS_FATAL("Please provide parameter world!");
        return 1;
    }
    if (!ros::param::get("wheelchair", wheelchair_id)) {
        ROS_FATAL("Please provide parameter wheelchair!");
        return 1;
    }
    char s[100];
    snprintf(s, sizeof(s), "%s/worlds/%s/wheelchairs/%d/pose",
             server_uri.data(), world_id.data(), wheelchair_id);
    pose_get_url = cpr::Url {s};
    snprintf(s, sizeof(s), "%s/worlds/%s/wheelchairs/%d",
             server_uri.data(), world_id.data(), wheelchair_id);
    pose_put_url = cpr::Url {s};
    double to;
    pnh.param<double>("timeout", to, 5.0);
    int64_t toi = round(1000 * to);
    timeout = cpr::Timeout {toi};
    pnh.param<double>("period", period, 1.0);
    ROS_INFO("Initialized with urls %s (GET) and %s (PUT) and timeout %ld s",
             pose_get_url.data(), pose_put_url.data(), toi);
    pose_pub = n.advertise<PoseWithCovarianceStamped>("initial_pose", 1);
    publish_initial_pose();
    ros::Subscriber sub = n.subscribe("pose", 1, send_update_pose_request);
    ros::spin();
    return 0;
}
