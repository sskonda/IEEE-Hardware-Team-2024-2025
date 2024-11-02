from picamera import PiCamera
from time import sleep

def read_Cam(file):

    camera = PiCamera()
    camera.start_preview(alpha= 192)
    sleep(1)
    camera.capture(file)
    camera.stop-preview()