# IEEE Robotics Hardware Team Software 2024-2025

## Overview  
This repository contains the software for the IEEE Robotics Hardware Team's entry in the **Mining Mayhem** competition. Our robot uses a **Raspberry Pi 4** as the main microcontroller and a **Pi Camera 2** to detect **AprilTags** for localization and task execution. To enhance situational awareness and precise movement, we use **ultrasonic sensors** on all four sides of the robot to detect nearby obstacles and maintain safe distances.  

The competition involves autonomously navigating a field, recognizing AprilTags, manipulating objects, and completing specific objectives to earn points.  

---

## Project Structure  

```
/root  
│  
├── src/  
│   ├── camera_control.py         # Interfaces with Pi Camera to detect AprilTags  
│   ├── april_tag_detection.py    # Detects AprilTags and extracts their IDs  
│   ├── navigation.py             # Controls movement and navigation based on AprilTags and ultrasonic sensors  
│   ├── object_manipulation.py    # Controls robot actions (picking, placing objects)  
│   ├── ultrasonic_control.py     # Reads ultrasonic sensor data for obstacle avoidance  
│   └── start_signal.py           # Detects match start LED or handles manual start switch  
│  
├── config/  
│   ├── hardware_config.yaml      # Hardware pin mappings and GPIO configurations  
│   └── april_tag_positions.yaml  # Known AprilTag positions on the field  
│  
├── tests/  
│   ├── test_camera_feed.py       # Verifies the camera setup and tag detection  
│   ├── test_navigation.py        # Simulates movement based on AprilTag input and sensor data  
│   ├── test_object_pickup.py     # Tests object manipulation logic  
│   └── test_ultrasonic.py        # Verifies ultrasonic sensor readings and accuracy  
│  
└── README.md                     # This file  
```

---

## Dependencies  
Ensure the following dependencies are installed:  

```bash
sudo apt update  
sudo apt install python3 python3-pip  
pip3 install opencv-python numpy apriltag-python RPi.GPIO  
```

---

## System Components  

1. **Raspberry Pi 4**:  
   Acts as the main controller, handling camera input, ultrasonic sensors, logic processing, and motor control.  

2. **Pi Camera 2**:  
   Captures the field and detects AprilTags for localization and path planning.  

3. **Ultrasonic Sensors**:  
   Four sensors (one on each side) provide real-time distance measurements to prevent collisions and assist with smooth navigation.  

4. **Motors and Actuators**:  
   Drive the robot and perform object manipulation (e.g., grabbing Cosmic Shipping Containers).  

5. **Start LED Detection**:  
   Robot initiates autonomous mode upon detecting the LED signal or via a manual switch as a fallback.  

---

## Code Overview  

- **`camera_control.py`**:  
   Initializes the Pi Camera, captures frames, and sends them to the AprilTag detection module.  

- **`april_tag_detection.py`**:  
   Detects AprilTags and extracts their IDs using OpenCV and AprilTag libraries. The tag IDs guide the robot’s movements and tasks.  

- **`navigation.py`**:  
   Controls robot movement, integrating AprilTag input with ultrasonic sensor readings for collision avoidance and smoother path planning.  

- **`ultrasonic_control.py`**:  
   Reads data from the ultrasonic sensors, ensuring obstacle detection and maintaining safe distances during navigation.  

- **`object_manipulation.py`**:  
   Handles actions like picking up and placing Astral Materials and Shipping Containers.  

- **`start_signal.py`**:  
   Detects the match start LED signal and initiates the robot’s tasks. If the LED is not detected, the manual switch starts the robot.  

---

## Workflow  

1. **Startup**:  
   - Robot waits for the **Start LED** signal.  
   - If the LED is not detected within a few seconds, the manual switch can start the robot.  

2. **Navigation**:  
   - Robot navigates based on AprilTag detections and ultrasonic sensor inputs.  
   - Known AprilTag IDs:
     - **5**: North Wall  
     - **6**: South Wall  
     - **7**: East Wall  
     - **0–4**: Telemetry Tag indicating the preferred Rendezvous Pad.

3. **Obstacle Avoidance**:  
   - Ultrasonic sensors on all sides detect nearby objects and adjust the robot’s movement to prevent collisions.  

4. **Object Manipulation**:  
   - The robot identifies and moves **Cosmic Shipping Containers** and **Astral Materials** (Geodinium/Nebulite).  
   - It places Astral Materials into the appropriate containers for maximum points.  

5. **End of Match**:  
   - The robot ensures that all elements are correctly placed to maximize post-match scoring.  

---

## How to Run the Code  

1. Connect the Pi Camera 2, ultrasonic sensors, and power up the Raspberry Pi 4.  
2. Navigate to the project directory.  
3. Run the main script:  

```bash
python3 src/main.py  
```  

4. Use the **tests/** directory to verify individual components. Example:  

```bash
python3 tests/test_camera_feed.py  
python3 tests/test_ultrasonic.py  
```  

---

## Troubleshooting  

- **Camera not detected**:  
   Ensure the Pi Camera is properly connected and enabled in the Raspberry Pi settings (`raspi-config`).  

- **AprilTag detection errors**:  
   Verify that the camera is focused and the tags are properly oriented. Adjust lighting if necessary.  

- **Ultrasonic sensor issues**:  
   Check connections and GPIO pin mappings in `hardware_config.yaml`. Ensure that the sensors are oriented correctly.  

- **Manual switch start not working**:  
   Verify GPIO configurations and ensure the switch is properly connected.  

---

## Future Improvements  

- Implement **SLAM** for dynamic localization if AprilTags are blocked during gameplay.  
- Use additional sensors (e.g., IMU) to further improve robot localization.  
- Optimize the robot’s path planning to reduce execution time and improve performance.  

---

## References  

- [AprilTag Library](https://april.eecs.umich.edu/software/apriltag)  
- Raspberry Pi GPIO Documentation: https://gpiozero.readthedocs.io  
