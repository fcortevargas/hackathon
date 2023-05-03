import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import json
import http.server
import socketserver
import threading

objectMarkers = {
    1: "robot1",
    2: "robot2",
    3: "element1",
    4: "element2",
    5: "element3"
}
terrainWidth = 20
terrainHeight = 20

positions = {};


class MarkerDetector:

    def __init__(self, camera):
        super()

        # OpenCV and aruco init
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;0"
        self.cap = cv2.VideoCapture(camera)
        self.key = getattr(aruco, 'DICT_4X4_250')
        self.arucoDict = aruco.getPredefinedDictionary(self.key)
        self.arucoParam = aruco.DetectorParameters_create()

        # Init class properties
        self.positions = {}
        self.terrainCorners = [None, None, None, None]
        self.isCalibrated = False
        self.warpMat = None

        self.projectedCorners = [[0, 0], [0, 1000], [1000, 0], [1000, 1000]]

        # Needed to allow OpenCV to wake up
        time.sleep(3)


    def calibrate(self, show=True):
        while self.terrainCorners.count(None) != 0:
            success, img = self.cap.read()
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # Detect aruco markers in image
            boxes, ids, rejected = aruco.detectMarkers(gray, self.arucoDict)

            if show:
                aruco.drawDetectedMarkers(img, boxes)
                cv2.imshow('calibrate', img)

            if ids is not None:
                for id, box in zip(ids, boxes):
                    if id <= 3:
                        print("Detected corner", id[0])
                        # Update corresponding corner location
                        self.terrainCorners[id[0]] = [int(box[0][0][0]), int(box[0][0][1])]

        terrainCornersArray = np.array(self.terrainCorners).astype(np.float32)
        cameraCornersArray = np.array(self.projectedCorners).astype(np.float32)

        self.warpMat = cv2.getPerspectiveTransform(terrainCornersArray, cameraCornersArray)

    def updatePositions(self, show=True):
        # Get image from camera
        success, img = self.cap.read()

        # Warp image to project to terrain
        warped = cv2.warpPerspective(img, self.warpMat, (1000, 1000))
        gray = cv2.cvtColor(warped, cv2.COLOR_BGR2GRAY)

        # Detect aruco markers in image
        boxes, ids, rejected = aruco.detectMarkers(gray, self.arucoDict)

        # Display if enabled (doesn't seem to work, probably due to ros)
        if show:
            aruco.drawDetectedMarkers(warped, boxes)
            cv2.imshow('img', warped)

        # Update list of positions
        if ids is not None:
            for id, box in zip(ids, boxes):
                if id > 3:
                    # if not self.isCalibrated(): continue;

                    # Calculate center of marker
                    x = [int(box[0][0][0]), int(box[0][1][0]), int(box[0][2][0]), int(box[0][3][0])]
                    y = [int(box[0][0][1]), int(box[0][1][1]), int(box[0][2][1]), int(box[0][3][1])]
                    center = (sum(x)/4, sum(y)/4)

                    # Calculate angle of marker
                    angle = np.arctan2((y[0]+y[1]/2)-center[1], (x[0]+x[1]/2)-center[0])

                    # TODO

                    # Create marker
                    marker = (
                        center[0],
                        center[1],
                        angle, # TODO: Calculate orientation properly
                        time.time()
                    )
                    self.positions[id[0]] = marker

        return self.positions

    def getPositions(self):
        return positions

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()


arucoDetector = MarkerDetector(4)

arucoDetector.calibrate()
while cv2.waitKey(1) != ord('q'):
    arucoDetector.updatePositions()


