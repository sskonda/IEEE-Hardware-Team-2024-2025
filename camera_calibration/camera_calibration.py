import cv2
import numpy as np
import glob

chessboard_size = (8, 6)  
square_size_x = 1  
square_size_y = 1  

objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, 0] = np.tile(np.arange(chessboard_size[0]) * square_size_x, chessboard_size[1])
objp[:, 1] = np.repeat(np.arange(chessboard_size[1]) * square_size_y, chessboard_size[0])

objpoints = []
imgpoints = []

images = glob.glob('*.jpg')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        objpoints.append(objp)
        refined_corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                           (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(refined_corners)

        cv2.drawChessboardCorners(img, chessboard_size, refined_corners, ret)
        cv2.imshow('Chessboard Detection', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

if len(objpoints) > 0 and len(imgpoints) > 0:
    ret, camera_matrix, distortion_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    np.savez("camera_calibration_data.npz", camera_matrix=camera_matrix, distortion_coeffs=distortion_coeffs)

    print("\nCamera Matrix:\n", camera_matrix)
    print("\nDistortion Coefficients:\n", distortion_coeffs)
    print(f"\nFocal Lengths: fx = {fx}, fy = {fy}")
    print(f"Principal Point: cx = {cx}, cy = {cy}")

else:
    print("Calibration failed: No chessboard corners detected in any images.")

def validate_focal_length(image_path, known_distance, real_width):
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:
        x_min, y_min = np.min(corners, axis=0)[0]
        x_max, y_max = np.max(corners, axis=0)[0]
        perceived_width = x_max - x_min  

        estimated_focal_length = (perceived_width * known_distance) / real_width

        print(f"Estimated Focal Length: {estimated_focal_length:.2f} pixels")
        print(f"Calibrated fx: {fx}, fy: {fy}")

        error = abs(estimated_focal_length - fx) / fx * 100
        print(f"Error in Calibration: {error:.2f}%")

    else:
        print("Chessboard not found in image!")

#validate_focal_length("calibration_image.jpg", known_distance=12.5, real_width=6 * square_size_x)
