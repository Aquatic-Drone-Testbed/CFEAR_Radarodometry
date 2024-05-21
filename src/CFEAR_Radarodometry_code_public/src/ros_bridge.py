import roslibpy
import time

def main():
    # Connect to ROS2 computer's rosbridge server
    host_ip = '192.168.1.111'   # PI-1 in ce lab
    # host_ip = '10.223.0.5'      # PI-1 in outside
    ros = roslibpy.Ros(host=host_ip, port=9092)
    ros.run()

    # Define the topic you want to publish to in ROS2
    topic = roslibpy.Topic(ros, '/NewTopic', 'std_msgs/String')

    # Create a sample message
    message = roslibpy.Message({'data': 'Hello from ROS1'})

    try:
        while ros.is_connected:
            # Publish the message
            topic.publish(message)
            print('Message published to ROS2')
            time.sleep(1)  # Adjust the sleep time as needed

    except KeyboardInterrupt:
        print('Interrupted!')

    finally:
        # Cleanup
        topic.unadvertise()
        ros.terminate()

if __name__ == '__main__':
    main()