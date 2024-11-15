#!/bin/python3
import rospy
from std_msgs.msg import String
import serial

def signal_publisher():
    rospy.init_node("signalnode", anonymous=True)
    startPub = rospy.Publisher('startSignalTopic', String, queue_size=1)

    # Set up the serial connection
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()

    # rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            start_sensor_input = ser.readline().decode('utf-8').strip()
            # rospy.logwarn(f"Message recived from p1:{start_sensor_input}")

            sensor_msg = String()

            # Logic based on the single sensor input
            if "Object Detected" == start_sensor_input or "1" == start_sensor_input:
                sensor_msg.data = "START"
                rospy.loginfo("*** Object detected, publishing START ***")
            elif "No Object" == start_sensor_input or "0" == start_sensor_input:
                sensor_msg.data = "IDLE"
                rospy.loginfo("*** No object detected, publishing IDLE ***")
            else:
                sensor_msg.data = "STOP"
                rospy.logwarn(f"Unexpected condition with Start Sensor Input: {start_sensor_input}")
                continue

            # Publish the message
            startPub.publish(sensor_msg)
            rospy.loginfo(f"Published signal: {sensor_msg.data}")

            # rate.sleep()

if __name__ == "__main__":
    try:
        signal_publisher()
    except rospy.ROSInterruptException:
        pass
