#!/usr/bin/env python3
"""
Subscribes to /ar_pose_marker (ar_track_alvar), converts the quaternion to Euler angles
and republishes a readable string on /marker_pose_info.
"""
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion


def quaternion_to_euler(orientation):
    quaternion = (orientation.x, orientation.y, orientation.z, orientation.w)
    roll, pitch, yaw = euler_from_quaternion(quaternion)
    return roll, pitch, yaw


def pose_callback(data):
    if len(data.markers) > 0:
        for marker in data.markers:
            position = marker.pose.pose.position
            orientation = marker.pose.pose.orientation
            roll, pitch, yaw = quaternion_to_euler(orientation)

            # 构建要发布的字符串信息
            pose_info = f"Marker ID: {marker.id}, Position -> x: {position.x}, y: {position.y}, z: {position.z}, Euler Angles -> Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}"
            rospy.loginfo(pose_info)

            # 发布位姿信息
            pub.publish(pose_info)
    else:
        rospy.loginfo("No markers detected")


def listener():
    global pub
    rospy.init_node('ar_pose_publisher', anonymous=True)

    # 创建一个发布者，发布到新话题 `/marker_pose_info`
    pub = rospy.Publisher('/marker_pose_info', String, queue_size=10)

    # 订阅 `/ar_pose_marker` 话题
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, pose_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException:
        pass


