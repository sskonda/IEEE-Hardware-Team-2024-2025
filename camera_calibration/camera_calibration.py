import cv2
import numpy as np
import glob
import os

# Chessboard configuration
chessboard_size = (8, 6)
square_size_m = (6+11/16) / 7 # Adjust to real-world size (inches or cm)
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

# Get all chessboard images (update path if needed)
images = glob.glob('input/*.jpg')

if not images:
    print(" No calibration images found! Make sure your images are in the 'images/' folder.")
    exit()

# Process each image
for idx, fname in enumerate(images):
    print(f"Processing {fname}...")

    img = cv2.imread(fname)
    if img is None:
        print(f" Warning: Could not load {fname}. Skipping...")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE )

    if ret:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(refined_corners)

        # Draw chessboard corners on the image (for debugging)
        cv2.drawChessboardCorners(img, chessboard_size, refined_corners, ret)

        # Save the processed image
        output_filename = fname.replace("input", "output")
        cv2.imwrite(output_filename, img)
        print(f" Chessboard detected! Saved as {output_filename}")
        time.sleep(1.0)
    else:
        print(f"Chessboard not found in {fname}. Skipping...")

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

# Validate Focal Length Function (Runs in Headless Mode)
def validate_focal_length(image_path, known_distance, real_width):
    """ Validate focal length by using an image with a known distance and object width. """
    if not os.path.exists(image_path):
        print(f" Error: Calibration image '{image_path}' not found.")
        return

    img = cv2.imread(image_path)
    if img is None:
        print(f" Warning: Could not load {image_path}.")
        return

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        x_min, y_min = np.min(corners, axis=0)[0]
        x_max, y_max = np.max(corners, axis=0)[0]
        perceived_width = x_max - x_min  

        estimated_focal_length = (perceived_width * known_distance) / real_width

        print(f"\n Estimated Focal Length: {estimated_focal_length:.2f} pixels")
        print(f" Calibrated fx: {fx}, fy: {fy}")

        error = abs(estimated_focal_length - fx) / fx * 100
        print(f"Error in Calibration: {error:.2f}%")

    else:
        print(" Chessboard not found in validation image!")
