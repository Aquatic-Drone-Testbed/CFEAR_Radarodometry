import rospy
import tf2_ros
from sensor_msgs.msg import Imu, NavSatFix, PointCloud2
from nav_msgs.msg import Odometry
import geometry_msgs.msg
import tf.transformations as transformations

def handle_imu(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "imu_link"

    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 1.0

    t.transform.rotation = msg.orientation

    br.sendTransform(t)

def handle_gps(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "base_link"
    t.child_frame_id = "gps_link"

    # Convert GPS coordinates to the appropriate frame coordinates
    t.transform.translation.x = msg.latitude  # Replace with appropriate conversion if needed
    t.transform.translation.y = msg.longitude  # Replace with appropriate conversion if needed
    t.transform.translation.z = msg.altitude  # Replace with appropriate conversion if needed

    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0
    t.transform.rotation.w = 1.0

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
#[TODO] change to correct topic names
    rospy.Subscriber('/imu/data', Imu, handle_imu)
    rospy.Subscriber('/gps/fix', NavSatFix, handle_gps)
    rospy.Subscriber('/radar_odom', Odometry, handle_odometry)
    rospy.Subscriber('/radar_registered', PointCloud2, handle_pointcloud2)

    rospy.spin()