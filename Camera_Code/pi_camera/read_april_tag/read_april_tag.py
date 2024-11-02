import cv2
import apriltag
import numpy

# load in the image

def readTag(file):

    image = cv2.imread(file, cv2.IMREAD_GRAYSCALE)

    median_blurred = cv2.medianBlur(image, 15)

    detector = apriltag.Detector()

    tags = detector.detect(median_blurred)

    return tags




