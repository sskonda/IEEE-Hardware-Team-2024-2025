import cv2
import apriltag
import numpy

import sys
sys.path.append('../')  # Go up one level; adjust as needed

import Capture_image_from_PiCam2.py  # Now you can import the file above

def main():
    # load in the image

    image = cv2.imread("testImage.jpg", cv2.IMREAD_GRAYSCALE)

    median_blurred = cv2.medianBlur(image, 15)

    detector = apriltag.Detector()

    tags = detector.detect(median_blurred)

    if tags:
        for tag in tags:
            print("Tag ID: ", tag.tag_id)
            print("Tag Corners: ", tag.corners)

    else:
        print("No AprilTag detected.")






