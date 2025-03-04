#!/bin/python3
import rospy
from std_msgs.msg import String
import serial

# Global queue for managing multiple cylinders
from collections import deque
cylinder_queue = deque()

# Counter for generating unique IDs for cylinders
cylinder_counter = 1

def signal_publisher():
    global cylinder_counter
    rospy.init_node("signalnode", anonymous=True)

    startPub = rospy.Publisher('startSignalTopic', String, queue_size=1)
    stopPub = rospy.Publisher('stopSignalTopic', String, queue_size=1)

    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            # Reading sensor data from serial port
            data = ser.readline().decode('utf-8').strip()

            # Assume data is in the format "Object detected,No object detected"
            try:
                start_sensor_input, stop_sensor_input = data.split(',')
            except ValueError:
                rospy.logwarn(f"Unexpected sensor input format: {data}")
                continue

            # Log the incoming data from both sensors for debugging
            rospy.logwarn(f"Received from sensors - PA: {start_sensor_input}, PB: {stop_sensor_input}")

            # Generate messages for proximity sensors
            sensor_msg = String()

            # Handle startProxiSensor detection (PA)
            if "Object detected" == start_sensor_input:
                # New cylinder detected at PA, generate a unique ID
                cylinder_id = f"Cylinder_{cylinder_counter}"
                cylinder_counter += 1

                # Add cylinder ID to the queue
                cylinder_queue.append(cylinder_id)

                # Publish a start message for the detected cylinder
                sensor_msg.data = f"START-{cylinder_id}"
                startPub.publish(sensor_msg)
                rospy.loginfo(f"Start Cylinder: {cylinder_id}")

            # Handle stopProxiSensor detection (PB)
            if "Object detected" == stop_sensor_input:
                # A cylinder is detected at PB, meaning it should now stop
                if cylinder_queue:
                    # Get the cylinder from the front of the queue
                    cylinder_id = cylinder_queue.popleft()
                    
                    # Publish a stop message for the detected cylinder
                    sensor_msg.data = f"STOP-{cylinder_id}"
                    stopPub.publish(sensor_msg)
                    rospy.loginfo(f"Stop Cylinder: {cylinder_id}")
                else:
                    rospy.logwarn("Stop sensor detected an object, but no cylinders are in the queue.")

            # Handle both sensors being idle
            if "No object detected" == start_sensor_input and "No object detected" == stop_sensor_input:
                rospy.loginfo("Both PA and PB are idle. No cylinders detected.")

if __name__ == "__main__":
    try:
        signal_publisher()
    except rospy.ROSInterruptException:
        pass
