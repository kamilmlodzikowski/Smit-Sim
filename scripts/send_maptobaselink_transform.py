#!/usr/bin/env python
import rospy

# Because of transformations
import tf

import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('tf2_broadcaster')
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.rotation.w = 1
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        t.header.stamp = rospy.Time.now()
        br.sendTransform(t)
        rate.sleep()