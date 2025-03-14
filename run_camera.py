#!/bin/python3
import numpy as np
from visualizer import draw_indicator, SERVER, OUTPUT
from robot import CAMERA, DRIVE, IMU
import pigpio
 

def compare_camera_motor_position(camera_position, camera_heading, motor_position, motor_heading):

    if camera_position is None or motor_position is None:
        print("One or both position estimates are missing.")
        return None

    # Compute Euclidean distance (position difference)
    position_difference = np.linalg.norm(np.array(camera_position) - np.array(motor_position))

    # Compute heading difference (normalize to range [-180, 180])
    heading_difference = (camera_heading - motor_heading + 180) % 360 - 180

    return position_difference, heading_difference


def main():
    PI = pigpio.pi()
    PI.wave_clear()

    CAMERA.init(PI)
    DRIVE.init(PI)
    IMU.init(PI)

    DRIVE.target_heading = None
    DRIVE.target_position = None

    SERVER.start()

    try:
        while True:
            DRIVE.update()
            IMU.update()
            CAMERA.update()
            DRIVE.current_heading[:] = IMU.current_heading[:]
            

            bg = np.zeros((500, 500, 3), np.uint8)
            frame = draw_indicator(bg, (0, 0), 0, color=(0, 255, 0))# green
            
            frame = draw_indicator(frame, DRIVE.current_position, DRIVE.current_heading, color=(255, 0, 0)) # blue 
            if CAMERA.detections:
                for detection in CAMERA.detections:
                    tag_id = detection["id"]  # get ID from detection 
                    print(f"Detected AprilTag ID: {tag_id}")
                
                camera_position, camera_heading = CAMERA.get_world_positon() or (None, None)
                if camera_position is not None and camera_heading is not None:
                    print("Position:", camera_position)
                    print("Heading (yaw):", camera_heading, "degrees")
                    
                    frame = draw_indicator(frame, camera_position, camera_heading, color=(255, 0, 255))# purple
                    
                    diff = compare_camera_motor_position(
                        camera_position, camera_heading, DRIVE.current_position, DRIVE.current_heading 
                    )

                    if diff is not None:
                        position_diff, heading_diff = diff

                        if position_diff is not None and heading_diff is not None:
                            print(f"Position Difference: {position_diff} inches")
                            print(f"Heading Difference: {heading_diff} degrees")

            OUTPUT.write(frame)

    except KeyboardInterrupt:
        pass

    finally:
        DRIVE.stop()
    

if __name__ == "__main__":
    main()
    