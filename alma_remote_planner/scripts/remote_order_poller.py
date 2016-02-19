#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Pose, Quaternion
import requests
import json
from types import ListType


def target2Pose(end):
    msg = Pose()
    msg.position.x = end[0] * 0.01
    msg.position.y = end[1] * 0.01
    # Move base accept PoseStamped targets, i.e. we must specify
    # an orientation.
    # When no orientation is provided, we pass yaw = 0 and rol, pitch != 0.
    # This way, the planners can check if a valid (2D) target angle is provided
    # and if not, they should issue a plan to travel towards a position,
    # ignoring the orientation (like the alma planner indeed does).
    if len(end) > 2:
        yaw = end[2]
        roll = 0
        pitch = 0
    else:
        yaw = 0
        roll = 1
        pitch = 1
    q = quaternion_from_euler(roll, pitch, yaw)
    msg.orientation = Quaternion(*q)
    print "->", msg
    return msg


def reset_target(url):
    response = requests.put(url, data=json.dumps({'order': {}}),
                            headers={'content-type': 'application/json'})
    rospy.loginfo(
        ("Sent request to reset the order to {url}. Got response {response}"
         .format(**locals())))
    if response.status_code == 200:
        current_target = None
        rospy.loginfo("Reset target")


def get_target(url, current_target):
    response = requests.get(url)
    rospy.loginfo(
        "Sent request to {url}. Got response {response}".format(**locals()))
    if response.status_code == 200:
        data = response.json()
        if 'order' in data:
            if data['order'] is None:
                if current_target is not None:
                    return (True, None)
                else:
                    return (False, None)
            if 'end' in data['order']:
                end = data['order']['end']
                rospy.loginfo("Got end {end}".format(**locals()))
                try:
                    end = json.loads(end)
                except Exception as e:
                    print e
                    end = None
                if end != current_target and type(end) is ListType:
                    return (True, end)
        else:
            if current_target is not None:
                return (True, None)
    return (False, current_target)


def main():
    rospy.init_node('poller_node', anonymous=False)
    url = rospy.get_param("~url", "")
    period = 1
    # Subscribe to the move_base action server
    move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

    # Wait 60 seconds for the action server to become available
    move_base.wait_for_server(rospy.Duration(60))

    rospy.loginfo("Connected to move base server")
    target = None
    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = 'map'
    while not rospy.is_shutdown():
        o_target = target
        (changed, target) = get_target(url, target)
        rospy.loginfo(
            "Changed {changed} to target {target}".format(**locals()))
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
        elif o_target:
            state = move_base.get_state()
            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Goal succeeded!")
                reset_target(url)
            elif state == GoalStatus.PREEMPTED:
                rospy.loginfo("Goal preempted!")
                reset_target(url)
        rospy.sleep(period)


if __name__ == '__main__':
    main()
