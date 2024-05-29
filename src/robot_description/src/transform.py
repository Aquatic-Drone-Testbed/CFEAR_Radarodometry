#!/usr/bin/env python
import rospy
import tf2_ros
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from nav_msgs.msg import Odometry
import geometry_msgs.msg

def handle_imu(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "bno055"

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 1.0

    t.transform.rotation = msg.orientation

    br.sendTransform(t)

def handle_odometry(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = "base_link"

    t.transform.translation.x = msg.pose.pose.position.x
    t.transform.translation.y = msg.pose.pose.position.y
    t.transform.translation.z = msg.pose.pose.position.z

    t.transform.rotation = msg.pose.pose.orientation

    br.sendTransform(t)

def handle_pointcloud2(msg):
    # PointCloud2 processing if needed
    pass

if __name__ == '__main__':
    rospy.init_node('dynamic_tf_broadcaster')

    # Static Transform Publisher
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = geometry_msgs.msg.TransformStamped()
    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "odom"
    static_transformStamped.child_frame_id = "base_link"

    static_transformStamped.transform.translation.x = 0.0
    static_transformStamped.transform.translation.y = 0.0
    static_transformStamped.transform.translation.z = 0.0
    static_transformStamped.transform.rotation.x = 0.0
    static_transformStamped.transform.rotation.y = 0.0
    static_transformStamped.transform.rotation.z = 0.0
    static_transformStamped.transform.rotation.w = 1.0

    static_broadcaster.sendTransform(static_transformStamped)

    # Subscribers
    rospy.Subscriber('/bno055/imu', Imu, handle_imu)
    # rospy.Subscriber('/gps/fix', NavSatFix, handle_gps)
    rospy.Subscriber('/cfear_radarodometry_node/radar_odom', Odometry, handle_odometry)
    rospy.Subscriber('/cfear_radarodometry_node/radar_registered', PointCloud2, handle_pointcloud2)

    rospy.spin()
