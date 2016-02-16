#include "ros/ros.h"
#include "json11.hpp"
#include <cpr/cpr.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace json11;

static cpr::Url pose_put_url;
static cpr::Url pose_get_url;

#define SERVER_TO_M 0.01
#define M_TO_SERVER_S 100000

static Json pose2json(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
        auto cov =  msg.pose.covariance;
        Json jcov =  Json::array {
                cov[0]*M_TO_SERVER_S,cov[1]*M_TO_SERVER_S,cov[5]*M_TO_SERVER_S,
                cov[6]*M_TO_SERVER_S,cov[7]*M_TO_SERVER_S,cov[11]*M_TO_SERVER_S,
                cov[30]*M_TO_SERVER_S,cov[31]*M_TO_SERVER_S,cov[35]*M_TO_SERVER_S
        };
        Json pose2D = Json::object {{"pose2D", Json::object {
                                             {"x", msg.pose.pose.position.x*SERVER_TO_M},
                                             {"y", msg.pose.pose.position.y*SERVER_TO_M},
                                             {"theta",  tf::getYaw(msg.pose.pose.orientation)},
                                             {"covariance",jcov}
                                     }}};
        return Json::object {{"ego_pose", pose2D}};
}

static geometry_msgs::PoseWithCovarianceStamped json2pose(const Json &value)
{
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        ///TODO: check if value has field 'covariance'
        auto cov = value["covariance"].array_items();
        msg.pose.covariance[0] = cov[0].number_value();
        msg.pose.covariance[1] = cov[1].number_value();
        msg.pose.covariance[5] = cov[2].number_value();
        msg.pose.covariance[6] = cov[3].number_value();
        msg.pose.covariance[7] = cov[4].number_value();
        msg.pose.covariance[11] = cov[5].number_value();
        msg.pose.covariance[30] = cov[6].number_value();
        msg.pose.covariance[31] = cov[7].number_value();
        msg.pose.covariance[35] = cov[8].number_value();
        Json pose_value = value["pose2D"];
        msg.pose.pose.position.x = pose_value["x"].number_value()*SERVER_TO_M;
        msg.pose.pose.position.y = pose_value["y"].number_value()*SERVER_TO_M;
        msg.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(pose_value["theta"].number_value());
        return msg;
}

static void send_update_pose_request(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
        Json data = pose2json(msg);
        auto r = cpr::Put(pose_put_url, cpr::Body {data.dump().data()}, cpr::Header {{"Content-Type", "application/json"}});
        ROS_INFO("Pose updated: %ld", r.status_code);
}

static void publish_initial_pose()
{
        ros::NodeHandle n;
        ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose", 1);
        auto r = cpr::Get(pose_get_url, cpr::Header {{"Content-Type", "application/json"}});
        if(r.status_code == 201)
        {
                std::string error;
                Json response = Json::parse(r.text,error);
                Json value = response["pose"];
                geometry_msgs::PoseWithCovarianceStamped pose = json2pose(value);
                pose_pub.publish(pose);
        }
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "remote_pose_updater");
        ros::NodeHandle n;
        n.param("get_url", pose_get_url, std::string(""));
        n.param("put_url", pose_put_url, std::string(""));
        publish_initial_pose();
        ros::Subscriber sub = n.subscribe("pose", 1, send_update_pose_request);
        ros::spin();
        return 0;
}
