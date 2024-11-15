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

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            # Read and decode the sensor input (assuming it receives data in the format "P1_value,P2_value")
            sensor_input = ser.readline().decode('utf-8').strip()
            rospy.loginfo(f"Received sensor input: {sensor_input}")

            try:
                # Extract values for P1 (Proximity Sensor 1) and P2 (Proximity Sensor 2)
                P1_value, P2_value = sensor_input.split(',')

                # Prepare the message to publish
                sensor_msg = String()

                # Logic for determining the signal state
                if (P1_value == "Object Detected" or P1_value == "1") and (P2_value == "No Object" or P2_value == "0"):
                    sensor_msg.data = "START"
                    rospy.loginfo("*** P1 detected object, P2 idle, publishing START ***")
                elif (P2_value == "Object Detected" or P2_value == "1") and (P1_value == "No Object" or P1_value == "0"):
                    sensor_msg.data = "STOP"
                    rospy.loginfo("*** P2 detected object, P1 idle, publishing STOP ***")
                else:
                    sensor_msg.data = "IDLE"
                    rospy.loginfo("*** No specific condition met, publishing IDLE ***")

                # Publish the message
                startPub.publish(sensor_msg)
                rospy.loginfo(f"Published signal: {sensor_msg.data}")

            except ValueError:
                rospy.logwarn(f"Unexpected sensor input format: {sensor_input}")

if __name__ == "__main__":
    try:
        signal_publisher()
    except rospy.ROSInterruptException:
        pass
