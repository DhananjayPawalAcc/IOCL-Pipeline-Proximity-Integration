#!/bin/python3
import numpy as np
from collections import OrderedDict
from scipy.spatial import distance as dist

class CentroidTracker:
	def __init__(self, maxDisappeared=50, maxDistance=50):
		"""
        DESCRIPTION:
			Initialize CentroidTracker class.
        ARGUMENTS:
			maxDisappeared (int): Max frames after which an object is considered as disappeared.
			maxDistance (int): Max distance for which same id is assigned to an object in consecutive frames.
        RETURNS:
        """
		self.nextObjectID = 0
		self.objects = OrderedDict()
		self.disappeared = OrderedDict()
		self.maxDisappeared = maxDisappeared
		self.maxDistance = maxDistance

	def register(self, centroid):
		"""
        DESCRIPTION:
			Register a centroid and assign it an object ID.
        ARGUMENTS:
			centroid (tuple): The centroid coordinates (x, y).
        RETURNS:
        """
		self.objects[self.nextObjectID] = centroid
		self.disappeared[self.nextObjectID] = 0
		self.nextObjectID += 1

	def deregister(self, objectID):
		"""
        DESCRIPTION:
			Deregister an object based on its ID.
        ARGUMENTS:
			objectID (int): The ID of the object to deregister.
        RETURNS:
        """
		del self.objects[objectID]
		del self.disappeared[objectID]

	def update(self, rects):
		"""
        DESCRIPTION:
			Update object centroids based on new bounding box detections.
        ARGUMENTS:
			rects (list): List of bounding boxes in the following format [[xmin1, ymin1, xmax1, ymin1], [xmin2, ymin2, xmax2, ymin2], ....]
        RETURNS:
			ids: collections.OrderedDict()
        """
		if len(rects) == 0:
			for objectID in list(self.disappeared.keys()):
				self.disappeared[objectID] += 1
				if self.disappeared[objectID] > self.maxDisappeared:
					self.deregister(objectID)
			return self.objects
		inputCentroids = np.zeros((len(rects), 2), dtype="int")
		for (i, (startX, startY, endX, endY)) in enumerate(rects):
			cX = int((startX + endX) / 2.0)
			cY = int((startY + endY) / 2.0)
			inputCentroids[i] = (cX, cY)
		if len(self.objects) == 0:
			for i in range(0, len(inputCentroids)):
				self.register(inputCentroids[i])
		else:
			objectIDs = list(self.objects.keys())
			objectCentroids = list(self.objects.values())
			D = dist.cdist(np.array(objectCentroids), inputCentroids)
			rows = D.min(axis=1).argsort()
			cols = D.argmin(axis=1)[rows]
			usedRows = set()
			usedCols = set()
			for (row, col) in zip(rows, cols):
				if row in usedRows or col in usedCols:
					continue
				if D[row, col] > self.maxDistance:
					continue
				objectID = objectIDs[row]
				self.objects[objectID] = inputCentroids[col]
				self.disappeared[objectID] = 0
				usedRows.add(row)
				usedCols.add(col)
			unusedRows = set(range(0, D.shape[0])).difference(usedRows)
			unusedCols = set(range(0, D.shape[1])).difference(usedCols)
			if D.shape[0] >= D.shape[1]:
				for row in unusedRows:
					objectID = objectIDs[row]
					self.disappeared[objectID] += 1
					if self.disappeared[objectID] > self.maxDisappeared:
						self.deregister(objectID)
			else:
				for col in unusedCols:
					self.register(inputCentroids[col])
		return self.objects
