#include "ros/ros.h"
#include "json11.hpp"
#include <cpr/cpr.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

using namespace json11;

static cpr::Url pose_put_url;
static cpr::Url pose_get_url;
static double timeout;
static ros::Publisher pose_pub;
static ros::Time last_ts;
static double period;

#define SERVER_TO_M 0.01
#define M_TO_SERVER_S 10000
#define M_TO_SERVER 100

static Json pose2json(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
        auto cov =  msg.pose.covariance;
        Json jcov =  Json::array {
                cov[0]*M_TO_SERVER_S,cov[1]*M_TO_SERVER_S,cov[5]*M_TO_SERVER_S,
                cov[6]*M_TO_SERVER_S,cov[7]*M_TO_SERVER_S,cov[11]*M_TO_SERVER_S,
                cov[30]*M_TO_SERVER,cov[31]*M_TO_SERVER,cov[35]
        };
        Json pose2D = Json::object {{"pose2D", Json::object {
                                             {"x", msg.pose.pose.position.x*M_TO_SERVER},
                                             {"y", msg.pose.pose.position.y*M_TO_SERVER},
                                             {"theta",  tf::getYaw(msg.pose.pose.orientation)}}},
                                    {"covariance", jcov}};
        return Json::object {{"ego_pose", pose2D}};
}

static geometry_msgs::PoseWithCovarianceStamped json2pose(const Json &value)
{
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.frame_id = "map";
        ///TODO: check if value has field 'covariance'
        std::string err;
        if(value.has_shape({{"covariance", Json::ARRAY}},err))
        {
                auto cov = value["covariance"].array_items();
                if(cov.size() ==9)
                {
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
        if(value.has_shape({{"pose2D", Json::OBJECT}},err))
        {
                Json pose_value = value["pose2D"];
                if(pose_value.has_shape({{"x", Json::NUMBER},{"y", Json::NUMBER},{"theta", Json::NUMBER}},err))
                {
                        msg.pose.pose.position.x = pose_value["x"].number_value()*SERVER_TO_M;
                        msg.pose.pose.position.y = pose_value["y"].number_value()*SERVER_TO_M;
                        msg.pose.pose.orientation =  tf::createQuaternionMsgFromYaw(pose_value["theta"].number_value());
                }
        }
        return msg;
}

static void send_update_pose_request(const geometry_msgs::PoseWithCovarianceStamped &msg)
{
  double delta_time = (msg.header.stamp - last_ts).toSec();
      if(delta_time > period)
      {
        last_ts = msg.header.stamp;
        Json data = pose2json(msg);
        auto r = cpr::Put(pose_put_url, cpr::Body {data.dump().data()}, cpr::Header {{"Content-Type", "application/json"}});
        ROS_INFO("Pose updated: %ld", r.status_code);
      }
}

static void publish_initial_pose()
{
        cpr::Response r = cpr::Get(pose_get_url, cpr::Header {{"Content-Type", "application/json"}},
                                   cpr::Timeout {(long)(1000 * timeout)});
        ROS_INFO("Fetched current pose. Got response %ld with content %s", r.status_code, r.text.data());
        if(r.status_code == 200)
        {
                std::string error;
                Json payload = Json::parse(r.text, error);
                if(!error.empty())
                {
                        ROS_INFO("Unable to parse response payload as json: %s", error.data());
                        return;
                }
                if(!payload.has_shape({{"pose", Json::OBJECT}}, error))
                {
                        ROS_INFO("Payload has not the right shape: %s", error.data());
                        return;
                }
                Json value = payload["pose"];
                geometry_msgs::PoseWithCovarianceStamped pose = json2pose(value);
                pose_pub.publish(pose);
                ROS_INFO("Could parse the payload and publish a pose");
        }
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "remote_pose_updater");
        ros::NodeHandle pnh("~");
        ros::NodeHandle n;
        last_ts = ros::Time(0);
        pnh.param<std::string>("get_url", pose_get_url, std::string(""));
        pnh.param<std::string>("put_url", pose_put_url, std::string(""));
        pnh.param<double>("timeout", timeout, 5.0);
        pnh.param<double>("period", timeout, 1.0);
        ROS_INFO("Initialized with urls %s (GET) and %s (PUT) and timeout %.1f s",
                 pose_get_url.data(),pose_put_url.data(),timeout);
        pose_pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initial_pose", 1);
        publish_initial_pose();
        ros::Subscriber sub = n.subscribe("pose", 1, send_update_pose_request);
        ros::spin();
        return 0;
}
