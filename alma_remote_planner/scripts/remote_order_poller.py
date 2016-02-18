#!/usr/bin/env python
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
import requests
import json
from types import ListType


def target2Pose(end):
    msg = Pose()
    msg.position.x = end[0]*0.01
    msg.position.y = end[1]*0.01
    msg.orientation = quaternion_from_euler(0,0,end[2])
    return msg

def reset_target(url):
    response = requests.post(url, data = json.dumps({'order': {}}))
    if response.status_code == 200:
        target = None
        rospy.loginfo("Reset target")

def get_target(url, current_target):
    response = requests.get(url)
    rospy.loginfo("Sent request to {url}. Got response {response}".format(**locals()))
    if response.status_code == 200:
        data = responde.json()
        if 'order' in data:
            if data['order'] is None and current_target is not None:
                return (True, None)
            if 'end' in data['order']:
                end = data['order']['end']
                rospy.loginfo("Got end {end}".format(**locals()))
                if end != current_target and type(end) is ListType:
                    return (True, end)
        else:
            if current_target is not None
                return (True, None)
    return (False, current_target)


def main():
    rospy.init_node('poller_node', anonymous=False)
    url = rospy.get_param("url", "")
    period = 10
    # Subscribe to the move_base action server
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # Wait 60 seconds for the action server to become available
    move_base.wait_for_server(rospy.Duration(60))

    rospy.loginfo("Connected to move base server")
    target = None
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    while not rospy.is_shutdown():
        (changed, target) = get_target(url, target)
        rospy.loginfo("Changed {changed} to target {target}".format(**locals()))
        if changed:
            if target:
                rospy.loginfo("Will send a new goal")
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = target2Pose(target)
                move_base.send_goal(goal)
                # if not move_base.wait_for_result():
                #     move_base.cancel_goal()
                #     rospy.loginfo("Timed out achieving goal")
                # else:
                #     # We made it!
                #     state = move_base.get_state()
                #     if state == GoalStatus.SUCCEEDED:
                #         rospy.loginfo("Goal succeeded!")
                #     elif state == GoalStatus.PREEMPTED:
                #         rospy.loginfo("Goal preempted!")
            else:
                rospy.loginfo("Will cancel goal")
                move_base.cancel_goal()
        rospy.sleep(period)


if __name__ == '__main__':
    main()
