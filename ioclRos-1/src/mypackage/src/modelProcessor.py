#!/bin/python3
import cv2 as cv
import rospy
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from utils.createConfig import create
import models
from mypackage.msg import detMsg, ocrMsg, camMsg, bboxMsg
from std_msgs.msg import Int64,String, Bool

def setModel():
    if cam_config['model_type'] == 'ocr':
            model = models.OCRProcessing(mod_config['craft_name'],mod_config['craft_protocol'], mod_config['craft_ip'], mod_config['craft_port'], mod_config['recognizor_name'], mod_config['recognizor_protocol'], mod_config['recognizor_ip'], mod_config['recognizor_port'], mod_config['ocr_border'], mod_config['ocr_min_bbox_area'], cam_config['disappeared'], cam_config['distance'], cam_config['detection_skip'])
    elif cam_config['model_type'] == 'detection':
            model = models.DetectionProcessing(mod_config['oring_yolo_name'], mod_config['oring_yolo_protocol'], mod_config['oring_yolo_ip'], mod_config['oring_yolo_port'], mod_config['oring_efficientad_name'], mod_config['oring_efficientad_protocol'], mod_config['oring_efficientad_ip'], mod_config['oring_efficientad_port'], mod_config['oring_threshold'], mod_config['pin_yolo_name'], mod_config['pin_yolo_protocol'], mod_config['pin_yolo_ip'], mod_config['pin_yolo_port'], mod_config['pin_efficientad_name'], mod_config['pin_efficientad_protocol'], mod_config['pin_efficientad_ip'], mod_config['pin_efficientad_port'], mod_config['pin_threshold'], mod_config['detection_border'], mod_config['detection_min_bbox_area'], cam_config['detection_skip'], cam_config['disappeared'], cam_config['distance'])
    return model


def callback(camMessage):
    frame = camMessage.image
    cvframe = Bridge.imgmsg_to_cv2(frame)
    output = model.predict(cvframe) # {'raw_image': image, 'result': results, 'drawn_image': drawn_image}
    if( PubmsgType==detMsg ):
        try :
          msg = detMsg()
          msg.header = camMessage.header

          msg.drawn_image = Bridge.cv2_to_imgmsg(output['drawn_image'])
          msg.raw_image = Bridge.cv2_to_imgmsg(output['raw_image'])

          msg.result.id = []
          for i in output['result']['id']:
                int_msg = Int64()
                if i is None:
                  int_msg.data = 1
                else:
                  int_msg.data = i
                msg.result.id.append(int_msg)

          msg.result.oring = [] 
          for i in output['result']['oring']:
             oring_msg = Bool()
             if i is None:
               oring_msg.data = False
             else:
                oring_msg.data= i
             msg.result.oring.append(oring_msg)
          
          msg.result.pin = [] 
          for i in output['result']['pin']:
             pin_msg = Bool()
             if i is None:
               pin_msg.data = False
             else:
                pin_msg.data = i
             msg.result.pin.append(pin_msg)

          msg.result.bbox.xmin.data = output['result']['bbox'][0][0] if len(output['result']['bbox'])>0 else 0
          msg.result.bbox.ymin.data = output['result']['bbox'][0][1] if len(output['result']['bbox'])>0 else 0
          msg.result.bbox.xmax.data= output['result']['bbox'][0][2] if len(output['result']['bbox'])>0 else 0
          msg.result.bbox.ymax.data = output['result']['bbox'][0][3] if len(output['result']['bbox'])>0 else 0

          pub.publish(msg)
          rospy.loginfo("Detection msg published")

        except Exception as e:
          rospy.loginfo(e)

    else:
      try:
          msg = ocrMsg() 
          msg.header = camMessage.header

          msg.drawn_image = Bridge.cv2_to_imgmsg(output['drawn_image'])
          msg.raw_image = Bridge.cv2_to_imgmsg(output['raw_image'])
         
          # Assigning to id (array of Int64)
          msg.result.id = []
          for i in output['result']['id']:
                int_msg = Int64()
                if i is None:
                  int_msg.data = 1
                else:
                  int_msg.data = i
                msg.result.id.append(int_msg)

         # Assigning to text (array of String)
          msg.result.text = []
          for t in output['result']['text']:
           str_msg = String()
           if t is None or t == "":
            str_msg.data = "NA"
           else:
             str_msg.data = t
           msg.result.text.append(str_msg)

          #list of bboxes
          msg.result.bbox = []
          bboxes = output['result']['bbox'] # ideally : [[237, 271, 525, 411], [239, 454, 592, 627]]
          if len(bboxes) == 1:
          # Handling a single bounding box case
            b1 = bboxes[0]
            bbox1 = bboxMsg()
            
            bbox1.xmin = Int64()
            bbox1.xmin.data = b1[0]
            
            bbox1.ymin = Int64()
            bbox1.ymin.data = b1[1]
            
            bbox1.xmax = Int64()
            bbox1.xmax.data = b1[2]
            
            bbox1.ymax = Int64()
            bbox1.ymax.data = b1[3]
    
            msg.result.bbox.append(bbox1)

            # Adding a dummy second bounding box as placeholder
            bbox2 = bboxMsg()
            
            bbox2.xmin = Int64()
            bbox2.xmin.data = 0
            
            bbox2.ymin = Int64()
            bbox2.ymin.data = 0
            
            bbox2.xmax = Int64()
            bbox2.xmax.data = 0
            
            bbox2.ymax = Int64()
            bbox2.ymax.data = 0
            
            msg.result.bbox.append(bbox2)

          elif len(bboxes) == 2:
              # Handling two bounding boxes case
              b1 = bboxes[0]
              bbox1 = bboxMsg()
              
              bbox1.xmin = Int64()
              bbox1.xmin.data = b1[0]
              
              bbox1.ymin = Int64()
              bbox1.ymin.data = b1[1]
              
              bbox1.xmax = Int64()
              bbox1.xmax.data = b1[2]
              
              bbox1.ymax = Int64()
              bbox1.ymax.data = b1[3]
              
              msg.result.bbox.append(bbox1)
          
              b2 = bboxes[1]
              bbox2 = bboxMsg()
              
              bbox2.xmin = Int64()
              bbox2.xmin.data = b2[0]
              
              bbox2.ymin = Int64()
              bbox2.ymin.data = b2[1]
              
              bbox2.xmax = Int64()
              bbox2.xmax.data = b2[2]
              
              bbox2.ymax = Int64()
              bbox2.ymax.data = b2[3]
              
              msg.result.bbox.append(bbox2)

          else:
              # Fallback when there are no bounding boxes
              bbox1 = bboxMsg()
              
              bbox1.xmin = Int64()
              bbox1.xmin.data = 0
              
              bbox1.ymin = Int64()
              bbox1.ymin.data = 0
              
              bbox1.xmax = Int64()
              bbox1.xmax.data = 0
              
              bbox1.ymax = Int64()
              bbox1.ymax.data = 0
              
              msg.result.bbox.append(bbox1)
          
              bbox2 = bboxMsg()
              
              bbox2.xmin = Int64()
              bbox2.xmin.data = 0
              
              bbox2.ymin = Int64()
              bbox2.ymin.data = 0
              
              bbox2.xmax = Int64()
              bbox2.xmax.data = 0
              
              bbox2.ymax = Int64()
              bbox2.ymax.data = 0
    
              msg.result.bbox.append(bbox2)

          pub.publish(msg)
          rospy.loginfo("OCR msg published")

      except Exception as e:
         rospy.loginfo(e)


   
   



if __name__=='__main__':
   #initialize the model processor node
   rospy.init_node("ModNode")
   rospy.Rate(30)

   camName = rospy.get_param("~name")
   modType = rospy.get_param("~model_type")

   # Create the config file object
   camConfigPath = 'src/mypackage/config/CamConfig.yaml'
   camConfig = create(camConfigPath)  # Dictionary
   cameras = camConfig["cameras"]  # List of  cam_dicts
   allCamDict = {
        camDict['name']:{
            'camera_type': camDict['camera_type'], 
            'model_type': camDict['model_type'], 
            'camera_id' : camDict['camera_id'],
            'frame_skip': camDict['frame_skip'], 
            'visualize_or_publish': camDict['visualize_or_publish'], 
            'distance': camDict['distance'], 
            'disappeared':  camDict['disappeared'],
            'detection_skip': camDict['detection_skip'], 
            'fps': camDict['fps'], 
            # 'ip': camDict['ip'],
            'endpoint': camDict['endpoint'],
            'port': camDict['port'],
            # 'wsip': camDict['wsip'],
            # 'wsport': camDict['wsport'],
            # 'required_bboxes':camDict['required_bboxes'],
            # 'priority':camDict['priority'],
            # 'exposure': camDict['exposure'], 
            # 'gain': camDict['gain'],
            # 'packet_size':camDict['packet_size'],
            # 'resize': camDict['resize'],
        } for camDict in cameras
    }
   cam_config = allCamDict[camName]
   
   modConfigPath = 'src/mypackage/config/ModelConfig.yaml'
   modConfig = create(modConfigPath)
   mod_config = modConfig[modType]
   
   #setting the model 
   model = setModel()
   Bridge = CvBridge()

   #creating the publisher
   PubmsgType = detMsg if cam_config['model_type']=='detection' else ocrMsg
   pubTopic = f"model/{camName}/{modType}"
   pub = rospy.Publisher(pubTopic,PubmsgType,queue_size=1)

   #creating the subscriber
   subTopic = f"{camName}/{modType}"
   rospy.Subscriber(subTopic,camMsg,callback)
   rospy.spin()