import numpy as np
import cv2 
from camera_calibration import fx, fy, cx, cy


#calibration coefficient based on the size of the object when known and the distance from the camera known 
calibration_coefficient = 5000

def calculate_distance(area):
    if area <= 0:
        return float('inf')  
    
    distance = calibration_coefficient / np.sqrt(area)
    return round(distance, 4) 

def calculate_3d_position(x_pixel, y_pixel, distance):
    X_c = (x_pixel - cx) / fx
    Y_c = (y_pixel - cy) / fy
    
    X = distance * X_c
    Y = distance * Y_c
    Z = distance
    return np.array([X, Y, Z])  # 3D position


# Function to detect objects in an image, specifically the purple balls using camera;
def detect_object(image_path):
    image = cv2.imread(image_path)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # purple color range
    lower_purple = np.array([125, 0, 0])
    upper_purple = np.array([175, 255, 255])
    mask = cv2.inRange(hsv, lower_purple, upper_purple)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 20:# Noise bounds but I dont know how big the pixel area is for the object
            continue

        x, y, w, h = cv2.boundingRect(contour)
        distance = calculate_distance(area)

        # Calc object's center coordinates
        x_center = x + w // 2
        y_center = y + h // 2

        object_position = calculate_3d_position(x_center, y_center, distance)

        #print("Object position: ", object_position)
        #print("Distance: ", distance)

        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

    # Show image
    cv2.imshow("Detected Object", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()