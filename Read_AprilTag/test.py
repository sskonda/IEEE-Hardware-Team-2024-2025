import cv2
import apriltag
import numpy

def main():
    print("Hello")

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




