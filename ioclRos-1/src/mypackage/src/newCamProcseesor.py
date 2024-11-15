#!/bin/python3
import rospy
import cv2 as cv
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import String
from collections import defaultdict
import uuid

# Dictionary to manage active cylinders and their buffers
active_cylinders = defaultdict(list)  # {cylinder_id: [frames]}

# Function to handle incoming start and stop messages
def signal_callback(msg):
    global active_cylinders
    message = msg.data
    action, cylinder_id = message.split('-')

    if action == "START":
        # Initialize a list to store frames for this cylinder if it's not already present
        if cylinder_id not in active_cylinders:
            active_cylinders[cylinder_id] = []
        rospy.loginfo(f"[INFO] Starting capture for cylinder {cylinder_id}")
    
    elif action == "STOP":
        # Save frames for this cylinder and then clear the buffer
        if cylinder_id in active_cylinders:
            rospy.loginfo(f"[INFO] Stopping capture for cylinder {cylinder_id}")
            # Here you can save frames or perform further processing
            # For example, save frames to disk or process them
            active_cylinders.pop(cylinder_id)

def start_camera_feed(camObj, camConfig, rate):
    global active_cylinders
    bridge = CvBridge()
    camera_id = camConfig['camera_id']
    pub = rospy.Publisher(f"{camConfig['name']}/camera_feed", Image, queue_size=1)
    
    while not rospy.is_shutdown():
        ret, frame = camObj.read()
        if ret:
            # Iterate over active cylinders and publish frames for each
            for cylinder_id in active_cylinders:
                # Store the frame in the buffer
                active_cylinders[cylinder_id].append(frame)
                
                # Convert frame to ROS Image message and publish
                ros_img = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
                ros_img.header.stamp = rospy.Time.now()
                ros_img.header.frame_id = cylinder_id  # Include the cylinder_id in the header
                
                pub.publish(ros_img)
                rospy.loginfo(f"[INFO] Published frame for cylinder {cylinder_id}")

            rate.sleep()
        else:
            rospy.logwarn(f"[WARN] Could not read frame from camera {camera_id}")

if __name__ == "__main__":
    rospy.init_node("camProcessorNode")
    rate = rospy.Rate(10)  # Adjust as needed
    
    # Load camera configuration (assumes 'camConfig' is already defined)
    ConfigPath = 'src/mypackage/config/CamConfig.yaml'
    config = create(ConfigPath)
    camConfig = config["cameras"][0]  # Modify this if you have multiple cameras
    
    camObj = set_camera(camConfig)
    
    # Subscribe to the start and stop topic to control the camera feed for cylinders
    rospy.Subscriber("startSignalTopic", String, signal_callback)
    rospy.Subscriber("stopSignalTopic", String, signal_callback)

    # Start the camera feed and frame publishing process
    start_camera_feed(camObj, camConfig, rate)
