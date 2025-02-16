import picamera2
from . import Component

from apriltag import apriltag
import cv2
import numpy as np
import picamera2
import sys
import time

 
class Camera(Component):
    def __init__(self, camera: int):
        super().__init__()
        self.picam2 = picamera2.Picamera2(camera)
        self.detector = apriltag("tag36h11")
    
    def init(self, pi):
        config = self.picam2.create_video_configuration({'format': 'RGB888'})
        self.picam2.configure(config)
        self.picam2.start()
    
    def get_frame(self) -> np.ndarray:
        self.frame = self.picam2.capture_array()
        return self.frame

    def detect_apriltag(self):
        s = time    
        im = self.get_frame()
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        # For some reason detector fails without this
        sys.stdout.write("\n")
        return self.detector.detect(im)

    def release(self):
        # Immpletement
        self.picam2.stop()
        pass