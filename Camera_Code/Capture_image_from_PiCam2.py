from picamera import PiCamera

from time import sleep
camera = PiCamera()
camera.start_preview(alpha= 192)
sleep(1)
camera.capture("python-apriltag/python-apriltag/images/testIMG.jpg")
camera.stop-preview()