import cv2
import numpy as np
import glob
import os
import pigpio
import time
import itertools

from visualizer import *
from robot import CAMERA

if __name__ == "__main__":
    NUM_FRAMES = 30

    CAMERA.init(pigpio.pi())

    if not CAMERA.initialized:
        print(" No calibration images found! Make sure your images are in the 'images/' folder.")
        exit()

    try:
        SERVER.start()

        chessboard_size = (8, 6)
        square_size_x = (6+11/16) / 7 # Adjust to real-world size (inches or cm)
        square_size_y = (4+11/16) / 5
        
        # Prepare real-world object points
        objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
        objp[:, 0] = np.tile(np.arange(chessboard_size[0]) * square_size_x, chessboard_size[1])
        objp[:, 1] = np.repeat(np.arange(chessboard_size[1]) * square_size_y, chessboard_size[0])
        
        # Storage for object points & image points
        objpoints = []  # 3D points in real-world space
        imgpoints = []  # 2D points in image plane
        
        # Ensure output directory exists
        if not os.path.exists("output"):
            os.makedirs("output")
        
        if not os.path.exists("input"):
            os.makedirs("input")
        
        # Process each image
        try:
            for idx in itertools.count():
                filename = f"frame_{idx}.jpg"
                print(f"Captured {filename}...")
                img = CAMERA.get_frame()
                OUTPUT.write(CAMERA.raw_frame)	   # Chessboard configuration
                img = cv2.resize(img, (0, 0), fx = 0.5, fy = 0.5)
        
                gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE )
        
                if ret:
                    cv2.imwrite(f"input/{filename}", img)
                    output_filename = f"output/{filename}"
        
                    objpoints.append(objp)
                    refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
                    imgpoints.append(refined_corners)
        
                    # Draw chessboard corners on the image (for debugging)
                    cv2.drawChessboardCorners(img, chessboard_size, refined_corners, ret)
        
                    # Save the processed image
                    cv2.imwrite(output_filename, img)
                    print(f" Chessboard detected! Saved as {output_filename}")
                    time.sleep(1.0)
                else:
                    print(f"Chessboard not found in {filename}. Skipping...")
        except KeyboardInterrupt:
            pass
        
        # Run calibration
        if objpoints and imgpoints:
            print("📏 Running camera calibration...")
            ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(
                objpoints, imgpoints, gray.shape[::-1], None, None
            )
        
            # Extract parameters
            fx = camera_matrix[0, 0]
            fy = camera_matrix[1, 1]
            cx = camera_matrix[0, 2]
            cy = camera_matrix[1, 2]
        
            # Save calibration results
            np.savez("camera_calibration_data.npz", camera_matrix=camera_matrix, distortion_coeffs=distortion_coeffs)
        
            print("\n Camera Calibration Complete!")
            print("\n Camera Matrix:\n", camera_matrix)
            print("\n Distortion Coefficients:\n", distortion_coeffs)
            print(f"\n Focal Lengths: fx = {fx}, fy = {fy}")
            print(f" Principal Point: cx = {cx}, cy = {cy}")
        
        else:
            print("> Calibration failed: No chessboard corners detected in any images.")
        
    except KeyboardInterrupt:
        pass
    finally:
        CAMERA.release()

