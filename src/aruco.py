import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time

objectMarkers = {
    1: "robot1",
    2: "robot2",
    3: "element1",
    4: "element2",
    5: "element3"
}
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
        self.arucoParam = aruco.DetectorParameters()
        self.positions = {}
        self.startTime = time.time()
        for v in objectMarkers.values():
            positions[v] = None
        time.sleep(3)

    def updatePositions(self, show=True):
        success, img = self.cap.read()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        boxes, ids, rejected = aruco.detectMarkers(gray, self.arucoDict, parameters=self.arucoParam)
        if show:
            aruco.drawDetectedMarkers(img, boxes)
            cv2.imshow('img', img)
        if ids is not None:
            for id,box in zip(ids, boxes):
                if id[0] == 0:  # Position marker : determines edge of field
                    pass
                else:  # Object marker : determines
                    if id[0] in objectMarkers.keys():
                        positions[objectMarkers[id[0]]] = {
                            "fl": [int(box[0][0][0]), int(box[0][0][1])],
                            "fr": [int(box[0][1][0]), int(box[0][1][1])],
                            "bl": [int(box[0][2][0]), int(box[0][2][1])],
                            "br": [int(box[0][3][0]), int(box[0][3][1])],
                            # "time": round(time.time() - self.startTime, 3)
                        }
        return positions

    def getPositions(self):
        return positions

    def __del__(self):
        self.cap.release()
        cv2.destroyAllWindows()