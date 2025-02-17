import picamera2
from . import Component

from apriltag import apriltag
import cv2
import numpy as np
import picamera2
import sys
import time

 
class Camera(Component):
    def __init__(self, camera: int):
        super().__init__()
        self.picam2 = picamera2.Picamera2(camera)
        self.load_calibration()
        self.detector = apriltag("tag36h11")
    
    def init(self, pi):
        config = self.picam2.create_video_configuration({'format': 'RGB888'})
        self.picam2.configure(config)
        self.picam2.start()
    
    def get_frame(self) -> np.ndarray:
        frame = self.picam2.capture_array()
        # if calibration data is available, undistort the frame
        if self.camera_matrix is not None and self.distortion_coeffs is not None:
            frame = self.undistort_frame(frame)
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
        return self.detector.detect(im)

    def release(self):
        # Immpletement
        self.picam2.stop()
        pass
    
    #Write pose matrix to self.pose_matrix
    #get pose using AprilTag Pose Estimation
    def pose_matrix(self,tag= 3.125 ):
        detections= self.detect_Apriltag()
        object_points = np.array([ # object points of the april tag
        [-tag / 2, -tag / 2, 0], 
        [ tag / 2, -tag / 2, 0],
        [ tag / 2,  tag / 2, 0],
        [-tag / 2,  tag / 2, 0]
    ], dtype=np.float32)

        for detection in detections: 
            image_points = np.array(detection["corners"], dtype=np.float32)
            #Use to solve pnp to get success,rotation, translation, 
            success, rvec, tvec = cv2.solvePnP(object_points, image_points, self.camera_matrix, self.distortion_coeffs)
            if success:
            # Convert rotation vector to rotation matrix
                rmat, _ = cv2.Rodrigues(rvec)
            # Construct 4x4 pose matrix
            self.pose_matrix = np.eye(4) #     Return a 2-D array with ones on the diagonal and zeros elsewhere.
            self.pose_matrix[:3, :3] = rmat
            self.pose_matrix[:3, 3] = tvec.flatten()
            print(f"Pose matrix for tag {detection['id']}:\n{self.pose_matrix}") # print to console the pose for each tag
        return self.pose_matrix
    