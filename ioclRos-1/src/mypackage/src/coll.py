#!/bin/python3
import re
import time
import uuid
import json
import cv2 as cv
import numpy as np
import random as rn
import requests as r
import datetime as dt
import collections as c
from utils.createConfig import create
import rospy
from cv_bridge import CvBridge
from std_msgs.msg import String, Bool
from mypackage.msg import detMsg, ocrMsg
import queue
import time,math

class Collator():
    def __init__(self, collator_dicts, Collargs,queues):
        """
        DESCRIPTION:
            Initialize the Collator object.
        ARGUMENTS:
            logger (logging.Logger): Logger object for logging events and logs for debugging.
            exit_event (threading.Event): Event object used to stop camera processor thread when KeyboardInterrupt is detected.
            args (dict): Configuration dict containing global parameters for all camera processors.
            collator_dicts (dict): Dictionary of parameters set in configuration file for collator.
        RETURNS:
        """
        #self.logger = logger
        self.args = Collargs
        self.collator_dicts = collator_dicts
        self.subTopiclist=[]
        self.bridge = CvBridge()
        self.queues = queues
        self.subscribers = []
        self.ocr_buffer2 = queue.Queue()
        self.ocr_buffer1= queue.Queue()
        self.det_buffer = queue.Queue()
         # Initialize subscribers for each camera in collator_dicts
        self.subDict = { }
        for cam_id in collator_dicts:
            cam_config = collator_dicts[cam_id]
            name = cam_config['cameraName']
            model = cam_config['model_type']
            topic_name = f"/model/{name}/{model}"
            msg_type = detMsg if model == 'detection' else ocrMsg

            self.subDict[name] = { 'name' :name,
                                   'model' : model,
                                   'topicName' : topic_name,
                                   'msgType' : msg_type
            }
        ap = [ name for name in self.subDict.keys()]
        rospy.Subscriber( self.subDict[ap[0]]['topicName'],self.subDict[ap[0]]['msgType'], lambda msg: self.callback(msg,self.subDict[ap[0]]['name']))
        rospy.Subscriber( self.subDict[ap[1]]['topicName'],self.subDict[ap[1]]['msgType'], lambda msg: self.callback(msg,self.subDict[ap[1]]['name']))
        rospy.Subscriber( self.subDict[ap[2]]['topicName'],self.subDict[ap[2]]['msgType'], lambda msg: self.callback(msg,self.subDict[ap[2]]['name']))
        rospy.Subscriber('signalTopic',Bool,self.listen)
        rospy.spin() 

    
    def listen(self,msg):
        if not msg.data:
            self.collate()
        else:
            print("fetching frames and filling buffer..")


    def callback(self,modelMsg,name):
        if type(modelMsg)==detMsg:
            Msg = {
             'time_stamp': modelMsg.header.stamp,
             'frame_id'  : modelMsg.header.frame_id,
             'raw_image':modelMsg.raw_image,
             'result'   : {
                 'bbox' : [ [modelMsg.result.bbox.xmin.data], [modelMsg.result.bbox.ymin.data], [modelMsg.result.bbox.xmax.data], [modelMsg.result.bbox.ymax.data] ],
                 'id'   : [id.data for id in modelMsg.result.id],
                 'oring': [oring.data for oring in modelMsg.result.oring],
                 'pin'  : [pin.data for pin in modelMsg.result.pin]
             },
               'drawn_image' : modelMsg.drawn_image
            }   
            self.queues[name]['queue'].put(Msg)
            self._fill_det_buffer(name)
           
        elif type(modelMsg)==ocrMsg:
            Msg = {
             'time_stamp': modelMsg.header.stamp,
             'frame_id'  : modelMsg.header.frame_id,
             'raw_image': modelMsg.raw_image,
             'result'   : {
                  'bboxes': [
                    [ bbox.xmin.data, bbox.ymin.data,bbox.xmax.data,bbox.ymax.data]
                    for bbox in modelMsg.result.bbox
                ],
                 'id'   : [id.data for id in modelMsg.result.id],
                 'text' : [str(text.data) for text in modelMsg.result.text],
             },
             'drawn_image' : modelMsg.drawn_image
            }
            self.queues[name]['queue'].put(Msg)
            self._fill_ocr_buffer(name)



    #buffer filling functions
    def _fill_ocr_buffer(self, name):
      if name =='b0_usb':
        if not self.queues[name]['queue'].empty():
         self.ocr_buffer1.put(self.queues[name]['queue'].get())  # Put an item from the queue into the buffer
      elif name =='b1_usb':
        if not self.queues[name]['queue'].empty():
         self.ocr_buffer2.put(self.queues[name]['queue'].get())  # Put an item from the queue into the buffer 

    def _fill_det_buffer(self, name):
      if name=='b2_gige':
       if not self.queues[name]['queue'].empty():
        self.det_buffer.put(self.queues[name]['queue'].get())  # Put an item from the queue into the buffer


    #collation function
    def collate(self):
         print("Starting collation process..")

         text_list1 = [msg['result'].get('text', "") for msg in list(self.ocr_buffer1.queue) if 'text' in msg['result']]
         final_list1  = []
         for i in text_list1:
             final_list1.extend(i)
         final_weight1, final_expiry1 = self._get_final_expiry_and_weight(final_list1)

         text_list2 = [msg['result'].get('text', "") for msg in list(self.ocr_buffer2.queue) if 'text' in msg['result']]
         final_list2  = []
         for i in text_list2:
             final_list2.extend(i)
         final_weight2, final_expiry2 = self._get_final_expiry_and_weight(final_list2)
         
         #get the best of the oring, pin from the detection buffer
         oring_list = [ msg['result'].get('oring',False) for msg in list(self.det_buffer.queue) if 'oring' in msg['result']]
         or_list = []
         for i in oring_list:
            or_list.extend(i)

         pin_list = [ msg['result'].get('pin',False) for msg in list(self.det_buffer.queue) if 'pin' in msg['result']]
         p_list = []
         for i in pin_list:
             p_list.extend(i)
 
         oring = self._voting(or_list)
         pin = self._voting(p_list)
         
         #get the best frame
         detBbox,detFrame = self.get_best_bbox_frame_det(self.det_buffer)
         ocrBbox1,ocrBbox2, ocrFrameA = self.get_best_bbox_frame_ocr(self.ocr_buffer1)
         ocrBbox_1, ocrBbox_2,ocrFrameB = self.get_best_bbox_frame_ocr(self.ocr_buffer2)

         # # Stitch the resultant images
         notification_frame = self.stitch_frames(detBbox, detFrame, ocrBbox1, ocrBbox2, final_weight1,final_expiry1,ocrFrameA, ocrBbox_1, ocrBbox_2, final_weight2,final_expiry2, ocrFrameB) # bbox2, bbox3, frame1,frame2,frame3)
        #  if not np.array_equal(notification_frame, np.zeros((640, 640, 3))):
         self.send_notification(notification_frame, oring, pin, final_weight1, final_expiry1, final_weight2, final_expiry2)
         print("NOTIFICATION SENT")
        #  else:
        #     pass
         self.reset_buffers()


    def get_best_bbox_frame_det(self,buffer):    
        bboxes = [ msg['result'].get('bbox',[]) for msg in list(buffer.queue) if 'bbox' in msg['result'] ]
        frames = [ msg['raw_image'] for msg in list(buffer.queue)]
        flattened_list1 = [[item[0] for item in sublist] for sublist in bboxes]
        
        #filtered1 = [item for item in flattened_list1 if item!=[0,0,0,0]]
        ideal_cordinates  = [ self.args['resize']//2, self.args['resize']//2 ]
        required_bbox=[]
        index = 0 
        min_distance  = float('inf')
        for  idx, bbox in enumerate(flattened_list1):
             if bbox!=[0,0,0,0]:
              x = bbox[0] + (bbox[2] - bbox[0]) // 2
              y = bbox[1] + (bbox[3] - bbox[1]) // 2
              d = math.dist(ideal_cordinates,[x,y])
              if d < min_distance:
                    min_distance = d 
                    required_bbox = bbox
                    index = idx
        if len(required_bbox)>0:
         frame = frames[index]
         frame = self.bridge.imgmsg_to_cv2(frame)
         return required_bbox, frame
        else:
            return [],np.zeros((640,640,3))
    
  
    def get_best_bbox_frame_ocr(self,buffer):
        bboxes = [ msg['result'].get('bboxes',[]) for msg in list(buffer.queue) if 'bboxes' in msg['result'] ] # 3D [[bbox1],[bbox2]]
        bboxes1 = [i[0] for i in bboxes]
        bboxes2 = [i[1] for i in bboxes] 

        frames = [ msg['raw_image'] for msg in list(buffer.queue)]

        ideal_cordinates  = [ self.args['resize']//2, self.args['resize']//2 ]
        index1 = 0
        index2 = 0
        required_bbox1 = []
        required_bbox2 = []
        min_distance1 =  float('inf')
        min_distance2  = 0

        for idx, bbox in enumerate(bboxes1):
             if bbox!=[0,0,0,0]:
              x = bbox[0] + (bbox[2] - bbox[0]) // 2
              y = bbox[1] + (bbox[3] - bbox[1]) // 2
              d = math.dist(ideal_cordinates,[x,y])
              if d < min_distance1:
                    min_distance1 = d 
                    required_bbox1 = bbox
                    index1 = idx

        for idx, bbox in enumerate(bboxes2):
             if bbox!=[0,0,0,0]:
              x = bbox[0] + (bbox[2] - bbox[0]) // 2
              y = bbox[1] + (bbox[3] - bbox[1]) // 2
              d = math.dist(ideal_cordinates,[x,y])
              if d > min_distance2:
                    min_distance2 = d 
                    required_bbox2 = bbox
                    index2 = idx

        i = max(index1,index2)
        frame = np.zeros((640,640,3))
        if i:
            frame = frames[i]
            frame = self.bridge.imgmsg_to_cv2(frame)
        return required_bbox1,required_bbox2,frame
    



    def stitch_frames(self, detBbox, detFrame, ocrBbox1, ocrBbox2, final_weight1, final_expiry1,ocrFrameA, ocrBbox_1, ocrBbox_2, final_weight2,final_expiry2, ocrFrameB):
          color1 = (0, 255, 0)  # Green for bounding box 1
          thickness = 2  # Thickness of the bounding box line
          font = cv.FONT_HERSHEY_SIMPLEX
          font_scale = 1.5
          font_thickness = 5
          text_color = (255, 255, 255)  # White text
    
        # if not np.array_equal(detFrame, np.zeros((640, 640, 3))) and \
        #     not np.array_equal(ocrFrameA, np.zeros((640, 640, 3))) and \
        #     not np.array_equal(ocrFrameB, np.zeros((640, 640, 3))):
            
            # Detected frame
          if len(detBbox) > 0 and not np.array_equal(detFrame,np.zeros((640,640,3))):
            cv.rectangle(detFrame, (detBbox[0], detBbox[1]), (detBbox[2], detBbox[3]), color1, thickness)
            #text_det = "Detection"
            #cv.putText(detFrame, text_det, (detBbox[0], detBbox[1] - 10), font, font_scale, text_color, font_thickness)

        # 2 bounding boxes on first OCR frame
          if len(ocrBbox1) > 0 and not np.array_equal(ocrFrameA,np.zeros((640,640,3))):
            cv.rectangle(ocrFrameA, (ocrBbox1[0], ocrBbox1[1]), (ocrBbox1[2], ocrBbox1[3]), color1, thickness)
            text_ocrA1 = final_expiry1
            cv.putText(ocrFrameA, text_ocrA1, (ocrBbox1[0], ocrBbox1[1] - 10), font, font_scale, text_color, font_thickness)

          if len(ocrBbox2) > 0 and not np.array_equal(ocrFrameA,np.zeros((640,640,3))):
            cv.rectangle(ocrFrameA, (ocrBbox2[0], ocrBbox2[1]), (ocrBbox2[2], ocrBbox2[3]), color1, thickness)
            text_ocrA2 = final_weight1
            cv.putText(ocrFrameA, text_ocrA2, (ocrBbox2[0], ocrBbox2[1] - 10), font, font_scale, text_color, font_thickness)

        # 2 bounding boxes on second OCR frame
          if len(ocrBbox_1) > 0 and not np.array_equal(ocrFrameB,np.zeros((640,640,3))):
            cv.rectangle(ocrFrameB, (ocrBbox_1[0], ocrBbox_1[1]), (ocrBbox_1[2], ocrBbox_1[3]), color1, thickness)
            text_ocrB1 = final_expiry2
            cv.putText(ocrFrameB, text_ocrB1, (ocrBbox_1[0], ocrBbox_1[1] - 10), font, font_scale, text_color, font_thickness)

          if len(ocrBbox_2) > 0 and not np.array_equal(ocrFrameB,np.zeros((640,640,3))):
            cv.rectangle(ocrFrameB, (ocrBbox_2[0], ocrBbox_2[1]), (ocrBbox_2[2], ocrBbox_2[3]), color1, thickness)
            text_ocrB2 = final_weight2
            cv.putText(ocrFrameB, text_ocrB2, (ocrBbox_2[0], ocrBbox_2[1] - 10), font, font_scale, text_color, font_thickness)

        # Combine OCR frames and detection frame into one stitched frame
          img1 = np.hstack((ocrFrameA, ocrFrameB))

          noImage = cv.imread('noImage.jpg')
          noImage = cv.resize(noImage, (640, 640))

          img2 = np.hstack((noImage, detFrame))
          frame = np.vstack((img1, img2))
          return frame
        # else:
        #   pass
        

    def send_notification(self,notification_frame,oring,pin,final_weight1,final_expiry1, final_weight2, final_expiry2):
        _, nf = cv.imencode('.jpg', notification_frame)
        anomaly_type = {'CylID': str(uuid.uuid1()), 'Oring': 'NO NOZZLE', 'Pin': 'NO NOZZLE', 'Weight': 'NO TEXT', 'ExpiryDate': 'NO TEXT'}
        anomaly_type['Oring'] = 'PRESENT' if oring else 'ABSENT'
        anomaly_type['Pin'] = 'OK' if pin else 'DAMAGED'

        o = False if anomaly_type['Oring']=='PRESENT' else True
        p = False if anomaly_type['Pin']=='OK' else True

        anomaly_type['Weight'] = final_weight1 or final_weight2
        anomaly_type['ExpiryDate'] = final_expiry1 or final_expiry2
        payload = {
            "timestamp": str(dt.datetime.now()),
            "anomaly_type": anomaly_type,
            "machine_id": rn.choice(['Conveyor belt 1', "Conveyor belt 2"]),
            "is_anomaly": o or p,
            "is_read": False
            }
        try:
         files = {
            "image": ("image.jpg",nf,"image/jpeg"),
            "data": (None, json.dumps(payload),"application/json")
         }
         response = r.post(self.args['notification_url'],files=files)
         response.raise_for_status()  # Will raise an HTTPError if the HTTP request returned an unsuccessful status code
        except Exception as e:
          rospy.logerr(f"Error sending notification: {e}")


    def value_correction(self, text):
        """
        DESCRIPTION:
            Correct text
        ARGUMENTS:
            text (str): String to be corrected.
        RETURNS:
            text: str
        """
        value = ""
        expiry_map= {
                '0': 'D',
                '3': 'B',
                '4': 'A',
                '6': 'C', 
                '8': 'B',
            }
        if len(text) >= 3 and len(text) < 5:
            if len(text) == 3 and text[0] == "1":
                if re.search('1[49]', text):
                    text = text[:2] + '.' + text[2:]
            elif len(text) == 3 and re.search("[03468]", text):
                first_digit = text[0]
                corrected_first_digit = expiry_map.get(first_digit, first_digit)
                corrected_text = corrected_first_digit + text[1:]
                corrected_text = corrected_text[:1] + '-' + corrected_text[1:]
                text = corrected_text
            elif re.search("[ABCD][0-9]{2}", text) and len(text) == 3:
                text = text[:1] + '-' + text[1:]
            else:
                text = ""
            value = text
        return value
    

    #function to get the best of ocr results
    def _get_final_expiry_and_weight(self, text_list):
        """
        DESCRIPTION:
            Select best text from list of texts and correct if needed
        ARGUMENTS:
            text_list (list): List of all texts.
        RETURNS:
            final_weight: str
            final_expiry: str
        """
        weights, expirys = [], []
        reject = []
        final_weight = ""
        final_expiry = ""
        for x in text_list:
            if len(x) >= 3 and len(x) < 5:
                if len(x) == 4 and x[0] == "1":
                    if re.search('1[49]\.[0-9]', x):
                        inner_dict = {}
                        inner_dict["score"] = 1.0
                        inner_dict["weight"] = x
                        final_weight = x
                        weights.append(inner_dict)
                elif len(x) == 3 and x[0] == "1":
                    if re.search('1[49]', x):
                        inner_dict = {}
                        inner_dict["score"] = 0.9
                        inner_dict["weight"] = x
                        weights.append(inner_dict)
                elif len(x) == 3 and re.search("[03468]", x):
                    inner_dict={}
                    inner_dict["score"] = 0.8
                    inner_dict["expiry"] = x
                    expirys.append(inner_dict)
                elif re.search("[ABCD][0-9]{2}",x) and len(x)==3:
                    inner_dict={}
                    inner_dict["score"] = 0.9
                    inner_dict["expiry"] = x
                    expirys.append(inner_dict)
                elif re.search("[ABCD]-[0-9]{2}",x) and len(x)==4:
                    inner_dict={}
                    inner_dict["score"] = 0.1
                    inner_dict["expiry"] = x
                    expirys.append(inner_dict)
                    final_expiry = x
                else:
                    reject.append(x)
            else:
                reject.append(x)
        if final_weight != "" and final_expiry != "":
            return final_weight, final_expiry
        elif final_weight != "" and final_expiry == "":
            if len(expirys) == 0:
                final_expiry = ""
            else:
                counts = c.Counter(item['expiry'] for item in expirys)
                max_count = max(counts.values())
                final_expiry = [expiry for expiry, count in counts.items() if count == max_count]
                final_expiry = self.value_correction(final_expiry[0])
            return final_weight,final_expiry
        elif final_weight == "" and final_expiry != "":
            if len(weights) == 0:
                final_weight = ""
            else:
                counts = c.Counter(item['weight'] for item in weights)
                max_count = max(counts.values())
                final_weight = [weight for weight, count in counts.items() if count == max_count]
                final_weight = self.value_correction(final_weight[0])
            return final_weight, final_expiry
        else:
            if len(weights) == 0:
                final_weight = ""
            else:
                counts = c.Counter(item['weight'] for item in weights)
                max_count = max(counts.values())
                final_weight = [weight for weight, count in counts.items() if count == max_count]
                final_weight = self.value_correction(final_weight[0])
            if len(expirys) == 0:
                final_expiry = ""
            else:
                counts = c.Counter(item['expiry'] for item in expirys)
                max_count = max(counts.values())
                final_expiry = [expiry for expiry, count in counts.items() if count == max_count]
                final_expiry = self.value_correction(final_expiry[0])
            if final_expiry != '' and int(final_expiry[-2:]) > self.args['max_year']:
                final_expiry = ''
            return final_weight, final_expiry
    
    #function to get detection result final
    def _voting(self, someBool_list):
        """
        DESCRIPTION:
            Select element with maximum occurance from list.
        ARGUMENTS:
            buffer (list): List of all elements
        RETURNS:
            winner: bool/float/int/str. Depends on type of list
        """
        unique = list(set(someBool_list))
        counts = [someBool_list.count(value) for value in unique]
        try:
            winner = unique[counts.index(max(counts))]
        except (IndexError,ValueError) as e:
            winner = [False]*4
        return winner
    

    #draw final result on the frame
    def _draw_final_decision_on_notification_frames(self, image, draw_dict):
        """
        DESCRIPTION:
            Draw final decision on image.
        ARGUMENTS:
            image (numpy.array): Image to draw on.
            draw_dict (dict): Shapes and text to draw.
        RETURNS:
            image: numpy.array
        """
        if len(draw_dict['expiry']) != 0:
            expiry_bbox, expiry = draw_dict['expiry']
            image = cv.rectangle(image, (expiry_bbox[0], expiry_bbox[1]), (expiry_bbox[2], expiry_bbox[3]), (0, 255, 0), 3)
            image = cv.putText(image, f'EXPIRY:{expiry}', (expiry_bbox[0], expiry_bbox[1] - 10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            image = image
        if len(draw_dict['weight'])!=0:
            weight_bbox, weight = draw_dict['weight']
            image = cv.rectangle(image, (weight_bbox[0], weight_bbox[1]), (weight_bbox[2], weight_bbox[3]), (0, 255, 0), 3)
            image = cv.putText(image, f'WEIGHT:{weight}', (weight_bbox[0], weight_bbox[1] - 10), cv.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        else:
            image = image
        return image
    
    # to empty the ocr and detection buffer
    def reset_buffers(self):
        self.ocr_buffer1.queue.clear()
        self.ocr_buffer2.queue.clear()
        self.det_buffer.queue.clear()
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    # Create the args file object
    ConfigPath = 'src/mypackage/config/CamConfig.yaml'
    camconfig = create(ConfigPath)  # Dictionary of list
    cameras = camconfig["cameras"]  # List of  cam_dicts

    # Create the args file object of collator
    ConfigPath = 'src/mypackage/config/CollConfig.yaml'
    collconfig = create(ConfigPath)  # Dictionary
    Collargs = collconfig["collator args"] 

    collator_dicts = {
        camera_config['name']: {
            "camera_id": camera_config['camera_id'],
            "cameraName" : camera_config['name'],
            "model_type": camera_config["model_type"],
            "required_bboxes": camera_config["required_bboxes"],
            "priority": camera_config["priority"],
            } for camera_config in cameras
        }
   
    #create a collator node
    rospy.init_node('collNode') #start the node
    rate = rospy.Rate(30)
    

    queues = { camera['name']:{
                    'queue':queue.Queue(),
                    'model_type':camera['model_type']
      }  for camera in cameras }
    
    collator = Collator(collator_dicts, Collargs, queues)
