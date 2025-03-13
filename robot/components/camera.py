import picamera2
from . import Component

from time import sleep
from apriltag import apriltag
import cv2
import numpy as np
import picamera2
import sys
import math

APRILTAG_POSE = { #inches, 0,0 top left corner, 0 degrees is east, world space 
    0: np.array([0.0, 22.5, 0.0]),
    1: np.array([0.0, 22.5, 0.0]),
    2: np.array([0.0, 22.5, 0.0]),
    3: np.array([0.0, 22.5, 0.0]),
    4: np.array([0.0, 22.5, 0.0]),
    5: np.array([32.0, 45.0, 270.0]),
    6: np.array([44.0, 0.0, 90.0]),
    7: np.array([93.0, 22.5, 180.0])
}
 
class Camera(Component):
    def __init__(self, camera: int = 0):
        super().__init__()
        
        self.camera_idx = camera
        self.detector = apriltag("tag36h11")
        self.detected = None
        self.pose_matrix = None
    
    def _init(self, pi):
        try:
            self.picam2 = picamera2.Picamera2(self.camera_idx)
            self.load_calibration()
            config = self.picam2.create_video_configuration({'format': 'RGB888'})
            self.picam2.configure(config)
            self.picam2.start()
            return True
        except Exception as e:
            print(f"Failed to initialize camera: {e}", file=sys.stderr)
            return False
    
    def get_frame(self) -> np.ndarray:
        frame = self.picam2.capture_array()
        # if calibration data is available, undistort the frame
        # if self.camera_matrix is not None and self.distortion_coeffs is not None:
        #     frame = self.undistort_frame(frame)
        return frame

    
    def load_calibration(self):
        calibration_file = "camera_calibration_data.npz"
        try:
            data = np.load(calibration_file)
            self.camera_matrix = data["camera_matrix"]
            self.distortion_coeffs = data["distortion_coeffs"]
            print("Camera calibration loaded successfully.")
        except FileNotFoundError:
            print(f"Calibration file '{calibration_file}' not found! Running without calibration.")
            self.camera_matrix = None
            self.distortion_coeffs = None    
            
    def undistort_frame(self, frame: np.ndarray) -> np.ndarray:
        h, w = frame.shape[:2]
        new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
            self.camera_matrix, self.distortion_coeffs, (w, h), 1, (w, h) #uses calibration data to get new camera matrix
        )
        undistorted_frame = cv2.undistort(frame, self.camera_matrix, self.distortion_coeffs, None, new_camera_matrix)
        x, y, w, h = roi
        undistorted_frame = undistorted_frame[y:y + h, x:x + w]
        return undistorted_frame      

    def detect_apriltag(self):
        im = self.get_frame()  # get frams undistorts them 
        im = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
        sys.stdout.write("\n") # DONT DELETE THIS LINE, NECESSARY FOR SOME REASON???
        detection = self.detector.detect(im)
        return detection

    def poll_for_light(self):
        N = 100
        brightness = np.empty(N)
        for i in range(N):
            im = self.get_frame()
            brightness[i] = np.mean(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
        
        std = np.std(brightness)
        mean = np.mean(brightness)
        print("Waiting for light...")
        
        try:
            while True:
                im = self.get_frame()
                brightness = np.mean(cv2.cvtColor(im, cv2.COLOR_BGR2GRAY))
                if brightness > mean + 100.0 * std:
                    break
        except KeyboardInterrupt:
            print("Manual Start.")
            pass
        
        return

    def _release(self):
        # Implement
        self.picam2.stop()
    
    #Write pose matrix to self.pose_matrix
    #get pose using AprilTag Pose Estimation
    def calculate_pose_matrix(self,tag= 3.125 ):
        detections= self.detect_apriltag()

        object_points = np.array([ # object points of the april tag
        [-tag / 2, -tag / 2, 0], 
        [ tag / 2, -tag / 2, 0],
        [ tag / 2,  tag / 2, 0],
        [-tag / 2,  tag / 2, 0]
    ], dtype=np.float32)

        for detection in detections: 
            image_points = np.array(detection["lb-rb-rt-lt"], dtype=np.float32)

            #Use to solve pnp to get success,rotation, translation, 
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.distortion_coeffs)
            if success:
            # Convert rotation vector to rotation matrix
                rmat, _ = cv2.Rodrigues(rvec)
            # Construct 4x4 pose matrix
            self.pose_matrix = np.eye(4) #     Return a 2-D array with ones on the diagonal and zeros elsewhere.
            self.pose_matrix[:3, :3] = rmat
            self.pose_matrix[:3, 3] = tvec.flatten()
        return self.pose_matrix

    def rotation_matrix_to_euler(R):
        sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = math.atan2(R[2, 1], R[2, 2])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = math.atan2(R[1, 0], R[0, 0])
        else:
            roll = math.atan2(-R[1, 2], R[1, 1])
            pitch = math.atan2(-R[2, 0], sy)
            yaw = 0

        return np.degrees((roll, pitch, yaw))

    def get_world_positon(self, pose_matrix, tag_id):

        if pose_matrix is None: 
            print("No pose matrix found")
            return None 

        if tag_id not in APRILTAG_POSE:
            print(f"Tag ID {tag_id} not found in APRILTAG_POSE dictionary")
            return None
    
    # get translation vectior 
        camera_position_vector = pose_matrix[:3, 3]
    #get rotation 
        rotation_position_vector = pose_matrix[:3, :3]

    # Get AprilTag's real-world position (x, y, z) in the field
        tag_world_position = APRILTAG_POSE[tag_id][:2] # x, y
        tag_world_position = APRILTAG_POSE[tag_id][2] # Extract the angle

        tag_rotation_degrees= np.degrees(tag_world_position)

        tag_rotation_matrix = np.array([
            [np.cos(tag_rotation_degrees), -np.sin(tag_rotation_degrees)],
            [np.sin(tag_rotation_degrees), np.cos(tag_rotation_degrees)]
        ])

        camera_global_xy = tag_world_position + tag_rotation_matrix@ camera_position_vector[:2]

        # have to convert the rotation to angle eurler, only yaw 
        pitch, roll, yaw = self.rotation_matrix_to_euler(rotation_position_vector)

        return (camera_global_xy, yaw)

