#!/bin/python3
import rospy
import cv2 as cv
import rospy
from cv_bridge import CvBridge
import basler as b
from sensor_msgs.msg import Image
from utils.createConfig import create
from mypackage.msg import camMsg
import datetime as dt
import uuid


def set_camera(camConfig):
        """
        Returns a cv2.VideoCapture() or basler.VideoCapture() class based on the camera_type.
        Also sets the exposure and gain value of the camera.
        """

        if camConfig['camera_type'] == 'opencv':
              if camConfig['camera_id'].startswith('opencv_'):
                camera = cv.VideoCapture(int(camConfig['camera_id'][7:]))
              else:
                camera = cv.VideoCapture(camConfig['camera_id'])
            
        else:  # Assuming 'basler' camera type
              if camConfig['camera_id'].startswith('basler_'):
                 camera = b.VideoCapture(int(camConfig['camera_id'][7:]))
              else:
                camera = b.VideoCapture(camConfig['camera_id'])
              camera.set_exposure(camConfig['exposure'])
              camera.set_gain(camConfig['gain'])
              camera.set_packet_size(camConfig['packet_size'])

        return camera


def startFeed_and_publish(camObj,camConfig,rate):
    exit_event = False
    bridge = CvBridge()
    camera_id = camConfig['camera_id']
    topic = f"{camConfig['name']}/{camConfig['model_type']}"
    pub = rospy.Publisher(topic,camMsg,queue_size=1)
    cam = camObj
    while not exit_event:
         
          ret,frame = cam.read()
          if ret : 
            msg = camMsg()
            msg.header.stamp =  rospy.Time.now()
            msg.header.frame_id = str(uuid.uuid4())
    
            frame = cv.resize(frame, (camConfig['resize'], camConfig['resize']))
            ros_img = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.image = ros_img

            print(f"[DEBUG] {camera_id} RECEIVED START SIGNAL")
            pub.publish(msg)
            rate.sleep()
          else:
            if not cam.isOpened():
                print(f"[ERROR] {camera_id} CANNOT OPEN CAMERA {camera_id}") 

if __name__=="__main__":
    # Create the config file object
    ConfigPath = 'src/mypackage/config/CamConfig.yaml'
    config = create(ConfigPath)  # Dictionary
    cameras = config["cameras"]  # List of  cam_dicts
  
    allCamDict = {
        camDict['name']:{
            'camera_id':camDict['camera_id'],
            'name':camDict['name'],
            'camera_type': camDict['camera_type'], 
            'model_type': camDict['model_type'], 
            'exposure': camDict['exposure'], 
            'gain': camDict['gain'],
            'packet_size':camDict['packet_size'],
            'resize': camDict['resize'],
        } for camDict in cameras
    }
    # for camdict in cameras:
    #     print(camdict)

    rospy.init_node("camNode")
    rate = rospy.Rate(30)
    nodeName = rospy.get_name()
    name = nodeName[1:]
    #configuration of the node cam to be started
    camConfig = allCamDict[name]
    camObj = set_camera(camConfig)
    startFeed_and_publish(camObj,camConfig,rate)



    