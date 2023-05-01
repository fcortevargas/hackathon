import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time

objectMarkers = {}
terrainWidth = 20
terrainHeight = 20

positions = {}


class ArucoDetector:

    def __init__(self, camera):
        super()
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;0"
        self.cap = cv2.VideoCapture(camera)
        self.key = getattr(aruco, 'DICT_4X4_250')
        self.arucoDict = aruco.getPredefinedDictionary(self.key)
        self.arucoParam = aruco.DetectorParameters_create()
        self.positions = {}
        self.startTime = time.time()
        time.sleep(3)

    def updatePositions(self, show=True):
        # Get image from camera
        success, img = self.cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Detect aruco markers in image
        boxes, ids, rejected = aruco.detectMarkers(gray, self.arucoDict)

        # Display if enabled (doesn't seem to work, probably due to ros)
        if show:
            aruco.drawDetectedMarkers(img, boxes)
            cv2.imshow('img', img)

        # Update list of positions
        if (ids is not None):
            for id, box in zip(ids, boxes):
                # Calculate center of marker
                x = [int(box[0][0][0]), int(box[0][1][0]), int(box[0][2][0]), int(box[0][3][0])]
                y = [int(box[0][0][1]), int(box[0][1][1]), int(box[0][2][1]), int(box[0][3][1])]
                center = (sum(x)/4, sum(y)/4)

                # Calculate angle of marker
                vec = ((x[0]+x[1]/2)-center[0], (y[0]+y[1]/2)-center[1])
                angle = np.angle(complex(vec[0],vec[1]), deg=True)

                # Create marker
                marker = (
                    center[0],
                    center[1],
                    angle, # TODO: Calculate orientation
                    round(time.time() - self.startTime,3)
                )
                positions[id[0]] = marker

        return positions

    def getPositions(self):
        return positions

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()

# d = ArucoDetector("rtsp://192.168.19.99:8086")
# d.updatePositions()