#!/usr/bin/env python3
"""
Minimal listener for /marker_pose_info (useful for checking ROS networking between machines).
"""
import rospy
from std_msgs.msg import String


def pose_info_callback(data):
    rospy.loginfo("Received Pose Info: %s", data.data)


def listener():
    rospy.init_node('ar_pose_subscriber', anonymous=True)

    # 订阅 `/marker_pose_info` 话题
    rospy.Subscriber('/marker_pose_info', String, pose_info_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
