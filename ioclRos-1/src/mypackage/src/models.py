#!/bin/python3
import cv2 as cv
import numpy as np
import tracker as t
import triton_client as tc
class Inferor:
    def __init__(self):
        """
        DESCRIPTION:
			Initialize Inferor class.
        ARGUMENTS:
        RETURNS:
        """
        pass

    def predict(self, image):
        """
        DESCRIPTION:
			Predict results from image input.
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
        RETURNS:
            result: dict
        """
        result = {'raw_image': image, 'result': {}, 'drawn_image': image}
        return result

    def track_objects(self, results):
        """
        DESCRIPTION:
            Track objects using results and update it.
        ARGUMENTS:
            results (dict): Results from predict() method.
        RETURNS:
            results: dict
        """
        return results

    def draw(self, image, results):
        """
        DESCRIPTION:
            Draw required shapes and text on image using results.
        ARGUMENTS:
            image (numpy.array): Image to be drawn on.
            results (dict): Results to be drawn.
        RETURNS:
            image: numpy.array
        """
        return image

class OCRProcessing(Inferor):
    def __init__(self, craft_name, craft_protocol, craft_ip, craft_port, recognizor_name, recognizor_protocol, recognizor_ip, recognizor_port, border, min_bbox_area, disappeared, distance, detection_skip): #logger):
        """
        DESCRIPTION:
			Initialize OCRProcessing class by inheriting the Inferor class.
        ARGUMENTS:
            craft_name (str): Name of craft model in Triton Inference Server.
            craft_protocol (str): Protocol of craft model in Triton Inference Server.
            craft_ip (str): IP address of craft model in Triton Inference Server.
            craft_port (int): Port of craft model in Triton Inference Server.
            recognizor_name (str): Name of recognizor model in Triton Inference Server.
            recognizor_protocol (str): Protocol of recognizor model in Triton Inference Server.
            recognizor_ip (str): IP address of recognizor model in Triton Inference Server.
            recognizor_port (int): Port of recognizor model in Triton Inference Server.
            border (int): Border for avoiding partial detections.
            min_bbox_area (int): Minimum area of bounding box to be considered.
            disappeared (int): Max frames after which an object is considered as disappeared by CentroidTracker class.
            distance (int): Max distance for which same id is assigned to an object in consecutive frames by CentroidTracker class.
            detection_skip (int): Index for which detection should be applied.
            logger (logging.Logger): Logger object for logging events and logs for debugging.
        RETURNS:
        """
        super(OCRProcessing, self).__init__()
        self.detector = tc.TritonModelSynchronous(craft_name, craft_protocol, craft_ip, craft_port) #logger)
        self.classifier = tc.TritonModelSynchronous(recognizor_name, recognizor_protocol, recognizor_ip, recognizor_port)# logger)
        self.tracker = t.CentroidTracker(disappeared, distance)
        self.border = border
        self.min_bbox_area = min_bbox_area
        self.detection_skip = detection_skip
        #self.logger = logger
        self.count = 0
        self.bbox_predictors = {}
        self.previous_results = {'bbox': [], 'id': [], 'text': []}

    def _preprocess(self, image):
        """
        DESCRIPTION:
            Preprocessing required to feed the detection model.
        ARGUMENTS:
            image (numpy.array): Image to run model on.
        RETURNS:
            image: numpy.array
        """
        image = cv.rotate(image, cv.ROTATE_90_CLOCKWISE)
        return image

    def _detect_texts_using_detector(self, image):
        """
        DESCRIPTION:
			Detect bounding boxes using craft model
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
        RETURNS:
            result: dict
        """
        results = {'bbox': [], 'id': [], 'text': []}
        image_h, image_w, _ = image.shape
        bboxes = self.detector.infer(image)[0].tolist()[0]
        for xmin, xmax, ymin, ymax in bboxes:
            area = (xmax - xmin) * (ymax - ymin)
            if self.border < xmin < image_w - self.border and self.border < ymin < image_h - self.border and self.border < xmax < image_w - self.border and self.border < ymax < image_h - self.border and area > self.min_bbox_area:
                results['bbox'].append([int(xmin), int(ymin), int(xmax), int(ymax)])
        return results

    def _initialize_bbox_predictors(self, image, results):
        """
        DESCRIPTION:
			Initialize bounding box predictor like CSRT by object id.
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
            results (dict): Result to give initial position of the object to be tracked.
        RETURNS:
            result: dict
        """
        for i, (xmin, ymin, xmax, ymax) in zip(results['id'], results['bbox']):
            bbox_predictor = cv.legacy.TrackerCSRT_create()
            success = bbox_predictor.init(image, (xmin, ymin, xmax - xmin, ymax - ymin))
            if success:
                self.bbox_predictors[i] = bbox_predictor
            else:
                self.bbox_predictors[i] = None
        return results

    def _detect_texts_using_bbox_predictor(self, image):
        """
        DESCRIPTION:
			Detect bounding box using bounding box predictor like CSRT by object id.
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
        RETURNS:
            result: dict
        """
        results = {'bbox': [], 'id': [], 'text': []}
        image_h, image_w, _ = image.shape
        for i in self.previous_results['id']:
            if i in self.bbox_predictors and self.bbox_predictors[i] is not None:
                success, (xmin, ymin, bbox_w, bbox_h) = self.bbox_predictors[i].update(image)
                area = bbox_w * bbox_h
                if self.border < xmin < image_w - self.border and self.border < ymin < image_h - self.border and self.border < xmin + bbox_w < image_w - self.border and self.border < ymin + bbox_h < image_h - self.border and area > self.min_bbox_area:
                    results['bbox'].append([int(xmin), int(ymin), int(xmin + bbox_w), int(ymin + bbox_h)])
        return results

    def track_objects(self, results):
        """
        DESCRIPTION:
            Track objects using results and update it.
        ARGUMENTS:
            results (dict): Results from predict() method.
        RETURNS:
            results: dict
        """
        identities = list(self.tracker.update(results['bbox']).items())
        ids = []
        for identity, (x, y) in identities:
            ids.append(identity)
        results['id'] = ids
        return results

    def _classify_text(self, crop):
        """
        DESCRIPTION:
            Recognize text from cropped images.
        ARGUMENTS:
            crop (numpy.array): Crop from detector or bounding box predictor to be fed to recognizor.
        RETURNS:
            text: str
        """
        try:
            text = self.classifier.infer(crop)[0].astype(str).tolist()[0]
        except Exception as e:
            #self.logger.debug(f'CLASSIFIER EXCEPTION: {e}')
            print(f'CLASSIFIER EXCEPTION: {e}')
            text = ''
        return text

    def draw(self, image, results):
        """
        DESCRIPTION:
            Draw required shapes and text on image using results.
        ARGUMENTS:
            image (numpy.array): Image to be drawn on.
            results (dict): Results to be drawn.
        RETURNS:
            image: numpy.array
        """
        for (xmin, ymin, xmax, ymax), text in zip(results['bbox'], results['text']):
            image = cv.rectangle(image, (int(xmin), int(ymin)), (int(xmax), int(ymax)), (0, 255, 0), 3)
            image = cv.putText(image, f'{text}', (xmin, ymin - 10), cv.FONT_HERSHEY_SIMPLEX, 3, (0, 255, 0), 3)
        return image

    def predict(self, image):
        """
        DESCRIPTION:
			Predict results from image input.
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
        RETURNS:
            result: dict
        """
        image = self._preprocess(image)
        raw_image = image.copy()
        if len(self.previous_results['id']) == 0 or self.count % self.detection_skip == 0:
            results = self._detect_texts_using_detector(image)
            results = self.track_objects(results)
            results = self._initialize_bbox_predictors(image, results)
        else:
            results = self._detect_texts_using_bbox_predictor(image)
            results = self.track_objects(results)
        for xmin, ymin, xmax, ymax in results['bbox']:
            crop = image[ymin: ymax, xmin: xmax, :]
            text = self._classify_text(crop)
            if len(text) < 3 or len(text) > 4 or text == '':
                results['bbox'].remove([xmin, ymin, xmax, ymax])
            else:
                results['text'].append(text)
        self.previous_results = results
        drawn_image = self.draw(image, results)
        self.count = self.count + 1
        return {'raw_image': raw_image, 'result': results, 'drawn_image': drawn_image}

class DetectionProcessing(Inferor):
    def __init__(self, oring_yolo_name, oring_yolo_protocol, oring_yolo_ip, oring_yolo_port, oring_efficientad_name, oring_efficientad_protocol, oring_efficientad_ip, oring_efficientad_port, oring_threshold, pin_yolo_name, pin_yolo_protocol, pin_yolo_ip, pin_yolo_port, pin_efficientad_name, pin_efficientad_protocol, pin_efficientad_ip, pin_efficientad_port, pin_threshold, border, min_bbox_area, detection_skip, disappeared, distance): #logger):
        """
        DESCRIPTION:
			Initialize DetectionProcessing class by inheriting the Inferor class.
        ARGUMENTS:
            oring_yolo_name (str): Name of oring YOLO model in Triton Inference Server.
            oring_yolo_protocol (str): Protocol of oring YOLO model in Triton Inference Server.
            oring_yolo_ip (str): IP address of oring YOLO model in Triton Inference Server.
            oring_yolo_port (int): Port of oring YOLO model in Triton Inference Server.
            oring_efficientad_name (str): Name of oring EfficientAD model in Triton Inference Server.
            oring_efficientad_protocol (str): Protocol of oring EfficientAD model in Triton Inference Server.
            oring_efficientad_ip (str): IP address of oring EfficientAD model in Triton Inference Server.
            oring_efficientad_port (int): Port of oring EfficientAD model in Triton Inference Server.
            oring_threshold (float): Threshold for classifying oring presence.
            pin_yolo_name (str): Name of pin YOLO model in Triton Inference Server.
            pin_yolo_protocol (str): Protocol of pin YOLO model in Triton Inference Server.
            pin_yolo_ip (str): IP address of pin YOLO model in Triton Inference Server.
            pin_yolo_port (int): Port of pin YOLO model in Triton Inference Server.
            pin_efficientad_name (str): Name of pin EfficientAD model in Triton Inference Server.
            pin_efficientad_protocol (str): Protocol of pin EfficientAD model in Triton Inference Server.
            pin_efficientad_ip (str): IP address of pin EfficientAD model in Triton Inference Server.
            pin_efficientad_port (int): Port of pin EfficientAD model in Triton Inference Server.
            pin_threshold (float): Threshold for classifying pin damage.
            border (int): Border for avoiding partial detections.
            min_bbox_area (int): Minimum area of bounding box to be considered.
            detection_skip (int): Index for which detection should be applied.
            disappeared (int): Max frames after which an object is considered as disappeared by CentroidTracker class.
            distance (int): Max distance for which same id is assigned to an object in consecutive frames by CentroidTracker class.
            logger (logging.Logger): Logger object for logging events and logs for debugging.
        RETURNS:
        """
        super(DetectionProcessing, self).__init__()
        self.oring_detector = tc.TritonModelSynchronous(oring_yolo_name, oring_yolo_protocol, oring_yolo_ip, oring_yolo_port)
        self.oring_classifier = tc.TritonModelSynchronous(oring_efficientad_name, oring_efficientad_protocol, oring_efficientad_ip, oring_efficientad_port) #logger=None)
        self.oring_threshold = oring_threshold
        self.pin_detector = tc.TritonModelSynchronous(pin_yolo_name, pin_yolo_protocol, pin_yolo_ip, pin_yolo_port)# logger=None)
        self.pin_classifier = tc.TritonModelSynchronous(pin_efficientad_name, pin_efficientad_protocol, pin_efficientad_ip, pin_efficientad_port)# logger=None)
        self.pin_threshold = pin_threshold
        self.border = border
        self.min_bbox_area = min_bbox_area
        self.detection_skip = detection_skip
        #self.logger = logger
        self.tracker = t.CentroidTracker(disappeared, distance)
        self.count = 0
        self.bbox_predictors = {}
        self.previous_results = {'bbox': [], 'id': [], 'oring': [], 'pin': []}

    def _detect_nozzle_using_detector(self, image):
        """
        DESCRIPTION:
			Detect bounding boxes using YOLO model
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
        RETURNS:
            result: dict
        """
        results = {'bbox': [], 'id': [], 'oring': [], 'pin': []}
        image_h, image_w, _ = image.shape
        bboxes = self.oring_detector.infer(image)[0].tolist()
        for xmin, ymin, xmax, ymax in bboxes:
            area = (xmax - xmin) * (ymax - ymin)
            if self.border < xmin < image_w - self.border and self.border < ymin < image_h - self.border and self.border < xmax < image_w - self.border and self.border < ymax < image_h - self.border and area > self.min_bbox_area:
                results['bbox'].append([int(xmin), int(ymin), int(xmax), int(ymax)])
        return results

    def _initialize_bbox_predictors(self, image, results):
        """
        DESCRIPTION:
			Initialize bounding box predictor like CSRT by object id.
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
            results (dict): Result to give initial position of the object to be tracked.
        RETURNS:
            result: dict
        """
        for i, (xmin, ymin, xmax, ymax) in zip(results['id'], results['bbox']):
            bbox_predictor = cv.legacy.TrackerCSRT_create()
            success = bbox_predictor.init(image, (xmin, ymin, xmax - xmin, ymax - ymin))
            if success:
                self.bbox_predictors[i] = bbox_predictor
            else:
                self.bbox_predictors[i] = None 
        return results

    def _detect_nozzle_using_bbox_predictor(self, image):
        """
        DESCRIPTION:
			Detect bounding box using bounding box predictor like CSRT by object id.
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
        RETURNS:
            result: dict
        """
        results = {'bbox': [], 'id': [], 'oring': [], 'pin': []}
        image_h, image_w, _ = image.shape
        for i in self.previous_results['id']:
            if i in self.bbox_predictors and self.bbox_predictors[i] is not None:
                success, (xmin, ymin, bbox_w, bbox_h) = self.bbox_predictors[i].update(image)
                area = bbox_w * bbox_h
                if self.border < xmin < image_w - self.border and self.border < ymin < image_h - self.border and self.border < xmin + bbox_w < image_w - self.border and self.border < ymin + bbox_h < image_h - self.border and area > self.min_bbox_area:
                    results['bbox'].append([int(xmin), int(ymin), int(xmin + bbox_w), int(ymin + bbox_h)])
        return results

    def track_objects(self, results):
        """
        DESCRIPTION:
            Track objects using results and update it.
        ARGUMENTS:
            results (dict): Results from predict() method.
        RETURNS:
            results: dict
        """
        identities = list(self.tracker.update(results['bbox']).items())
        ids = []
        for identity, (x, y) in identities:
            ids.append(identity)
        results['id'] = ids
        return results

    def _classify_oring_presence_and_pin_damage(self, nozzle):
        """
        DESCRIPTION:
            Classify oring presence and oring damage from nozzle crop.
        ARGUMENTS:
            nozzle (numpy.array): Crop from detector or bounding box predictor to be fed to oring EfficientAD and pin EfficientAD.
        RETURNS:
            oring_present: bool
            pin_ok: bool
        """
        oring_present = False
        pin_ok = False
        oring_heatmap = self.oring_classifier.infer(nozzle)[0]
        #self.logger.debug(f'ORING MAX: {np.max(oring_heatmap)}')
        #print(f'ORING MAX: {np.max(oring_heatmap)}')
        if np.max(oring_heatmap) < self.oring_threshold:
            oring_present = True
        # bboxes = self.pin_detector.infer(nozzle)[0].tolist()
        # if len(bboxes) > 0:
        # xmin, ymin, xmax, ymax = bboxes[0]
        # crop = nozzle[ymin:ymax, xmin:xmax, :]
        nozzle_h, nozzle_w, _ = nozzle.shape
        crop = nozzle[int(0.2 * nozzle_h): -1 * int(0.2 * nozzle_h), int(0.2 * nozzle_w):-1 * int(0.2 * nozzle_w)]
        # cv.imwrite('/test/crop.png', crop)
        pin_heatmap = self.pin_classifier.infer(crop)[0]
        #self.logger.debug(f'PIN MAX: {np.max(pin_heatmap)}')
        #print(f'PIN MAX: {np.max(pin_heatmap)}')
        if np.max(pin_heatmap) > self.pin_threshold:
            pin_ok = True
        return oring_present, pin_ok

    def draw(self, image, results):
        """
        DESCRIPTION:
            Draw required shapes and text on image using results.
        ARGUMENTS:
            image (numpy.array): Image to be drawn on.
            results (dict): Results to be drawn.
        RETURNS:
            image: numpy.array
        """
        for (xmin, ymin, xmax, ymax), oring_present, pin_ok in zip(results['bbox'], results['oring'], results['pin']):
            if oring_present:
                oring_present = 'PRESENT'
                color = (0, 255, 0)
            else:
                oring_present = 'ABSENT'
                color = (0, 0, 255)
            image = cv.putText(image, f'ORING: {oring_present}', (xmin, ymin - 10), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            if pin_ok:
                pin_ok = 'OK'
                color = (0, 255, 0)
            else:
                pin_ok = 'DAMAGED'
                color = (0, 0, 255)
            image = cv.putText(image, f'PIN: {pin_ok}', (xmin, ymax + 20), cv.FONT_HERSHEY_SIMPLEX, 1, color, 2)
            image = cv.rectangle(image, (xmin, ymin), (xmax, ymax), (255, 0, 0), 3)
        return image

    def predict(self, image):
        """
        DESCRIPTION:
			Predict results from image input.
        ARGUMENTS:
            image (numpy.array): Image to be used as input.
        RETURNS:
            result: dict
        """
        if len(self.previous_results['id']) == 0 or self.count % self.detection_skip == 0:
            results = self._detect_nozzle_using_detector(image) 
            results = self.track_objects(results)
            results = self._initialize_bbox_predictors(image, results)
        else:
            results = self._detect_nozzle_using_bbox_predictor(image)
            results = self.track_objects(results)
        for xmin, ymin, xmax, ymax in results['bbox']:
            nozzle = image[ymin: ymax, xmin: xmax, :]
            oring_present, pin_ok = self._classify_oring_presence_and_pin_damage(nozzle)
            results['oring'].append(oring_present)
            results['pin'].append(pin_ok)
        self.previous_results = results
        drawn_image = self.draw(image, results)
        self.count = self.count + 1
        return {'raw_image': image, 'result': results, 'drawn_image': drawn_image}
