import picamera2
from ..constants import *
from . import Component

from libcamera import Transform
from apriltag import apriltag
import cv2
import numpy as np
import picamera2
import sys
import math

class Camera(Component):
    def __init__(self, camera: int = 0):
        super().__init__()

        self.camera_idx = camera
        self.detector = apriltag("tag36h11")
        self.raw_frame = None
        self.frame = None
        self.frame_tick = None
        self.undistorted_frame = None
        self.detections = None
        self.pose_matrix = None

    def _init(self, pi):
        try:
            self.load_calibration()
            self.picam2 = picamera2.Picamera2(self.camera_idx)
            
            transform = CAMERA_CONFIGURATION.pop("transform", None)
            if transform is not None:
                CAMERA_CONFIGURATION["transform"] = Transform(**transform)

            config = self.picam2.create_video_configuration(
                {'format': 'RGB888'},
                **CAMERA_CONFIGURATION
            )
            self.picam2.configure(config)
            self.picam2.start()
            return True
        except Exception as e:
            print(f"Failed to initialize camera: {e}", file=sys.stderr)
            return False

    def update(self):
        self.get_frame()
        self.undistort_frame()
        self.detect_apriltags()
        self.calculate_pose_matrix()

    def load_calibration(self):
        calibration_file = "camera_calibration_data.npz"
        try:
            data = np.load(calibration_file)
            self.camera_matrix = data["camera_matrix"]
            self.distortion_coeffs = data["distortion_coeffs"]
            print("Camera calibration loaded successfully.")
        except FileNotFoundError:
            print(
                f"Calibration file '{calibration_file}' not found! Running without calibration.")
            self.camera_matrix = None
            self.distortion_coeffs = None

    def get_frame(self):
        frame_buffer = self.picam2.capture_array()
        if self.raw_frame is None:
            self.raw_frame = np.empty_like(frame_buffer)

        # copy frame buffer to self.frame
        self.raw_frame[:, :, :] = frame_buffer[:, :, :]
        self.frame_tick = self.pi.get_current_tick() if self.pi else None
        # if calibration data is available, undistort the frame
        # if self.camera_matrix is not None and self.distortion_coeffs is not None:
        #     frame = self.undistort_frame(frame)
        if self.frame is None:
            self.frame = np.empty_like(self.raw_frame)
        if np.all(self.frame.shape == self.raw_frame.shape):
            self.frame[:, :, :] = self.raw_frame[:, :, :]
        return self.raw_frame

    def undistort_frame(self):
        if self.raw_frame is not None:
            h, w = self.raw_frame.shape[:2]
            new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
                # uses calibration data to get new camera matrix
                self.camera_matrix, self.distortion_coeffs, (w, h), 1, (w, h)
            )
            undistorted_frame = cv2.undistort(
                self.raw_frame, self.camera_matrix, self.distortion_coeffs, None, new_camera_matrix)
            x, y, w, h = roi
            self.frame = undistorted_frame[y:y + h, x:x + w]
        return self.frame

    def detect_apriltags(self):
        if self.frame is None:
            return None

        im = cv2.cvtColor(self.frame, cv2.COLOR_BGR2GRAY)
        # DONT DELETE THIS LINE, NECESSARY FOR SOME REASON???
        sys.stdout.write("\n")

        self.detections = self.detector.detect(im)
        return self.detections

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
                if brightness > mean + 50.0 * std:
                    break
        except KeyboardInterrupt:
            print("Manual Start.")
            pass

        return

    def _release(self):
        # Implement
        self.picam2.stop()

    # Write pose matrix to self.pose_matrices
    def calculate_pose_matrix(self):
        if self.detections is None:
            return None

        image_points = []
        world_points = []
        for detection in self.detections:
            image_points.extend(
                detection["lb-rb-rt-lt"]
            )

            tag_world_location, tag_world_rotation = APRILTAG_POSE[detection["id"]]
            tag_world_rotation = np.radians(tag_world_rotation)
            world_points.extend(
                tag_world_location[None, :] +
                (np.array([[np.cos(tag_world_rotation), -np.sin(tag_world_rotation), 0.0],
                          [np.sin(tag_world_rotation),  np.cos(tag_world_rotation), 0.0],
                          [0.0,                         0.0,                        1.0]], dtype=np.float32) @ TAG_POINTS.T).T
            )
        print(world_points)

        # Use to solve pnp to get success,rotation, translation,
        image_points = np.array(image_points, dtype=np.float32)
        world_points = np.array(world_points, dtype=np.float32)

        if len(world_points) == 0 or len(image_points) == 0 or len(world_points) != len(image_points):
            self.pose_matrix = None
        else:
            success, rvec, tvec = cv2.solvePnP(
                world_points, image_points, self.camera_matrix, self.distortion_coeffs)
            if success:
                # Convert rotation vector to rotation matrix
                rmat, _ = cv2.Rodrigues(rvec)
                # Construct 4x4 pose matrix
                # Return a 2-D array with ones on the diagonal and zeros elsewhere.
                pose_matrix = np.eye(4)
                pose_matrix[:3, :3] = rmat
                pose_matrix[:3, 3] = tvec.flatten()
                self.pose_matrix = np.linalg.inv(pose_matrix)
            else:
                self.pose_matrix = None
            return self.pose_matrix

    def get_world_positon(self):

        if self.detections is None:
            print("No detections found")
            return None

        if self.pose_matrix is None:
            print("No pose matrix found")
            return None

        robot_camera = np.array([0.0, 0.0, 0.0, 1.0])
        robot_forward_camera = np.array([0.0, 0.0, 1.0, 0.0])

        robot_world = self.pose_matrix @ robot_camera
        robot_forward_world = self.pose_matrix @ robot_forward_camera
        
        heading = np.arctan2(robot_forward_world[1], robot_forward_world[0])

        return robot_world[:2], np.degrees(heading)
