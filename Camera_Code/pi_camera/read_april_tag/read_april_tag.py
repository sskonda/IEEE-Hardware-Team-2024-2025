import cv2
import apriltag
import numpy
import os

# load in the image

def readTag(file_path):

    current_dir = os.path.dirname(__file__)

    parent_dir = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

    file_path = os.path.join(parent_dir, file_path)

    image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)

    median_blurred = cv2.medianBlur(image, 15)

    detector = apriltag.Detector()

    tags = detector.detect(median_blurred)

    return tags




