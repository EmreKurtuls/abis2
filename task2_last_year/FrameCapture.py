import logging
import math
import threading
import time
import cv2

logging.basicConfig(filename='log.txt', level=logging.INFO, format='%(asctime)s - %(message)s')

class Camera:
    def __init__(self, videoCapturePort=0):
        self.videoCapturePort = videoCapturePort
        self.camera = self.openCamera()
        self.currentFrame = None
        self.isActive = True
        self.isCameraOpened = False
        self.cameraThread = threading.Thread(target=self.readCamera)

        self.trackedObjects = {}

        self.cameraThread.start()
        while not self.isCameraOpened:
            self.isCameraOpened = self.camera.isOpened()

    def openCamera(self):
        return cv2.VideoCapture(self.videoCapturePort)

    def readCamera(self):
        while self.isActive:
            ret, frame = self.camera.read()
            if ret:
                self.currentFrame = frame
            else:
                print("Failed to capture frame")
                self.currentFrame = None
        self.camera.release()

    def getCurrentFrame(self):
        return self.currentFrame


