import numpy as np
import cv2 
import os
import time
from picamera2 import Picamera2

# Check if calibration data exists before proceeding
calibration_file = "camera_calibration_data.npz"
if not os.path.exists(calibration_file):
    print(f"Calibration file '{calibration_file}' not found! Run camera calibration first.")
    exit()

# Load calibration data
data = np.load(calibration_file)

if "camera_matrix" not in data:
    print("Invalid calibration file! Ensure the calibration process was completed.")
    exit()
# Extract camera matrix
camera_matrix = data["camera_matrix"]

# Extract focal lengths and principal point
fx = camera_matrix[0, 0]  # Focal length in x-axis
fy = camera_matrix[1, 1]  # Focal length in y-axis
cx = camera_matrix[0, 2]  # Principal point x-coordinate
cy = camera_matrix[1, 2]  # Principal point y-coordinate
#calibration coefficient based on the size of the object when known and the distance from the camera known 
calibration_coefficient = 5000; # Formula: calibration_coefficient = (known_width * sqrt(known_area)) / known_distance= 

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

def detect_object(image_path):
    if not os.path.exists(image_path):
        print(f"Image file '{image_path}' not found!")
        return

    image = cv2.imread(image_path)
    
    if image is None:
        print(f"Error: Could not load image '{image_path}'")
        return

    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_purple = np.array([120, 80, 40]) 
    upper_purple = np.array([170, 255, 255])  

    mask = cv2.inRange(hsv, lower_purple, upper_purple)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    object_detected = False  # Flag to check if an object is found

    for contour in contours:
        area = cv2.contourArea(contour)
        if area < 20:
            continue

        x, y, w, h = cv2.boundingRect(contour)
        distance = calculate_distance(area)

        x_center = x + w // 2
        y_center = y + h // 2
        object_position = calculate_3d_position(x_center, y_center, distance)

        print(f"Object Position: {object_position}, Distance: {distance} cm")

        cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 2)
        cv2.drawContours(image, [contour], -1, (0, 255, 0), 2)

        object_detected = True

    if not object_detected:
        print("No objects detected in the image.")

    cv2.imshow("Detected Object", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

#detect stream from camera and detect purple objects real time
def detect_object_cam():
    picam2 = Picamera2()
    picam2.preview_configuration.main.size = (640, 480)
    picam2.preview_configuration.main.format = "RGB888"
    picam2.configure("preview")
    picam2.start()

    frame_count = 0  

    while True:
        frame = picam2.capture_array()
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)

        lower_purple = np.array([120, 80, 40])  
        upper_purple = np.array([170, 255, 255])  
        mask = cv2.inRange(hsv, lower_purple, upper_purple)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        object_detected = False

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 50:  
                continue

            x, y, w, h = cv2.boundingRect(contour)
            distance = calculate_distance(area)

            x_center = x + w // 2
            y_center = y + h // 2
            object_position = calculate_3d_position(x_center, y_center, distance)

            print(f"Object Position: {object_position}, Distance: {distance} cm")

            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cv2.drawContours(frame, [contour], -1, (0, 255, 0), 2)

            object_detected = True

        if not object_detected:
            print("No objects detected.")

        # Overwrite the same image files instead of saving new ones
        cv2.imwrite("frames/detection.jpg", frame)
        cv2.imwrite("frames/mask.jpg", mask)

        frame_count += 1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if not os.path.exists("frames"):
    os.makedirs("frames")

detect_object_cam()