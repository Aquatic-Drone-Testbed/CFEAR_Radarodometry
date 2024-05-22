import roslibpy
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2

'''
    # Connect to ROS2 computer's rosbridge server
    #host_ip = '192.168.1.111'   # PI-1 in ce lab
    # host_ip = '10.223.0.5'      # PI-1 in outside
'''


def odom_callback(data, topic):
    # Convert ROS1 message to dictionary
    message = {
        'header': {
            'seq': data.header.seq,
            'stamp': {'secs': data.header.stamp.secs, 'nsecs': data.header.stamp.nsecs},
            'frame_id': data.header.frame_id
        },
        'child_frame_id': data.child_frame_id,
        'pose': {
            'pose': {
                'position': {'x': data.pose.pose.position.x, 'y': data.pose.pose.position.y, 'z': data.pose.pose.position.z},
                'orientation': {'x': data.pose.pose.orientation.x, 'y': data.pose.pose.orientation.y, 'z': data.pose.pose.orientation.z, 'w': data.pose.pose.orientation.w}
            },
            'covariance': data.pose.covariance
        },
        'twist': {
            'twist': {
                'linear': {'x': data.twist.twist.linear.x, 'y': data.twist.twist.linear.y, 'z': data.twist.twist.linear.z},
                'angular': {'x': data.twist.twist.angular.x, 'y': data.twist.twist.angular.y, 'z': data.twist.twist.angular.z}
            },
            'covariance': data.twist.covariance
        }
    }
    topic.publish(roslibpy.Message(message))

def pointcloud_callback(data, topic):
    # Convert ROS1 message to dictionary
    message = {
        'header': {
            'seq': data.header.seq,
            'stamp': {'secs': data.header.stamp.secs, 'nsecs': data.header.stamp.nsecs},
            'frame_id': data.header.frame_id
        },
        'height': data.height,
        'width': data.width,
        'fields': [{'name': field.name, 'offset': field.offset, 'datatype': field.datatype, 'count': field.count} for field in data.fields],
        'is_bigendian': data.is_bigendian,
        'point_step': data.point_step,
        'row_step': data.row_step,
        'data': list(data.data),  # Convert byte array to list
        'is_dense': data.is_dense
    }
    topic.publish(roslibpy.Message(message))

def main():
    rospy.init_node('ros1_to_ros2_bridge')

    host_ip = 'localhost'  # Use the appropriate IP address for your setup
    ros = roslibpy.Ros(host=host_ip, port=9090)
    ros.run()

    odom_topic = roslibpy.Topic(ros, '/radar_odom', 'nav_msgs/Odometry')
    pointcloud_topic = roslibpy.Topic(ros, '/radar_registered', 'sensor_msgs/PointCloud2')

    rospy.Subscriber('/radar_odom', Odometry, odom_callback, odom_topic)
    rospy.Subscriber('/radar_registered', PointCloud2, pointcloud_callback, pointcloud_topic)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('Interrupted!')

    finally:
        odom_topic.unadvertise()
        pointcloud_topic.unadvertise()
        ros.terminate()

if __name__ == '__main__':
    main()


