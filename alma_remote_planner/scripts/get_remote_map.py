#!/usr/bin/env python
import rospy
import requests
import zipfile
import StringIO
import rospkg


def main():
    rospy.init_node('get_remote_map_node', anonymous=False)

    if not (rospy.has_param('server_uri') and rospy.has_param('world')):
        rospy.logfatal("no server_uri or world params provided")
        return

    server = rospy.get_param("server_uri", None)
    world = rospy.get_param("world", None)
    resolution = rospy.get_param("resolution", 5)

    url = ("{server}/worlds/{world}/map.gmapping?resolution={resolution}"
           .format(**locals()))

    rospack = rospkg.RosPack()
    # TODO: not nice
    path = "{root}/worlds/{world}/map".format(
        root=rospack.get_path('alma_remote_planner'),
        world=world)

    response = requests.get(url, stream=True)
    rospy.loginfo(
        "Sent request to {url}. Got response {response}".format(**locals()))
    if response.status_code != 200:
        rospy.logerror("No valid response")
    else:
        z = zipfile.ZipFile(StringIO.StringIO(response.content))
        z.extractall(path=path)

if __name__ == '__main__':
    main()
