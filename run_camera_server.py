#!/usr/bin/python3

# Mostly copied from https://picamera.readthedocs.io/en/release-1.13/recipes2.html
# Run this script, then point a web browser at http:<this-ip-address>:8000
# Note: needs simplejpeg to be installed (pip3 install simplejpeg).

import pigpio
from visualizer import *

from robot import CAMERA

CAMERA.init(pigpio.pi())


try:
    SERVER.start()
    while True:
        CAMERA.get_frame()
        OUTPUT.write(CAMERA.raw_frame)
except KeyboardInterrupt:
    pass
finally:
    CAMERA.release()
