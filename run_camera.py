#!/bin/python3
from robot.components import Camera
import numpy as np
from visualizer import draw_indicator, SERVER, OUTPUT

def transform_to_position_heading(matrix):
    """
    Converts a 4x4 transformation matrix into position (x, y, z)
    and heading (yaw angle in degrees).

    Parameters:
        matrix (np.ndarray): 4x4 transformation matrix.

    Returns:
        tuple: (position, heading)
        - position: (x, y, z)
        - heading: yaw angle in degrees
    """
    if matrix.shape != (4, 4):
        raise ValueError("Input matrix must be 4x4.")

    # Extract translation vector
    position = matrix[:3, 3]
    position = np.array([position[2], position[0]])
    
    # Extract rotation matrix
    rotation_matrix = matrix[:3, :3]

    # Yaw (heading) is the rotation around the Y-axis
    yaw = np.arctan2(rotation_matrix[0, 2], rotation_matrix[2, 2]) * (180.0 / np.pi)

    return (position, yaw)


def main():
    camera = Camera(0)
    camera.init(None)
    SERVER.start()

    try:
        while True:
            tag_pose_matrix = camera.calculate_pose_matrix()
            bg = np.zeros((500, 500, 3), np.uint8)
            origin = draw_indicator(bg, (0, 0), 0, color=(0, 255, 0))

            if tag_pose_matrix is None:
                OUTPUT.write(origin)
                continue
            camera_pose = np.linalg.inv(tag_pose_matrix)
            position, heading = transform_to_position_heading(camera_pose)

            print(f"Pose matrix for tag: \n{camera_pose}") # print to console the pose for each tag
            print("Position:", position)
            print("Heading (yaw):", heading, "degrees")

            robot = draw_indicator(origin, position, heading, color=(255, 0, 0))
            OUTPUT.write(robot)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
    