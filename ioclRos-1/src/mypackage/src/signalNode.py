#!/bin/python3
import rospy
from std_msgs.msg import Bool

def signal_publisher():
    rospy.init_node("signalnode", anonymous=True)
    pub = rospy.Publisher('signalTopic', Bool, queue_size=1)

    signal_msg = Bool()
    signal_msg.data = True  # Start with signal being True

    rate = rospy.Rate(0.2)  # Set the rate to 1 Hz (i.e., toggle signal every 1 second)

    while not rospy.is_shutdown():
        # Publish the current signal
        pub.publish(signal_msg)
        
        # Log the signal being sent
        rospy.loginfo(f"Published signal: {signal_msg.data}")
        
        # Toggle the signal
        signal_msg.data = not signal_msg.data
        
        # Sleep for the specified interval (1 second here)
        rate.sleep()

if __name__ == "__main__":
    try:
        signal_publisher()
    except rospy.ROSInterruptException:
        pass
