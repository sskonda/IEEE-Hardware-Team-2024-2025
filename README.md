# IEEE Robotics Hardware Team Software 2024-2025

## Overview  
This repository contains the software for the IEEE Robotics Hardware Team's entry in the **Mining Mayhem** competition. Our robot uses a **Raspberry Pi 4** as the main microcontroller and a **Pi Camera 2** to detect **AprilTags** for localization and task execution. The competition involves autonomously navigating a field, recognizing AprilTags, manipulating objects, and completing specific objectives to earn points.  

---

## Project Structure  
The project repository is organized as follows:  

```
/root  
│  
├── src/  
│   ├── camera_control.py         # Interfaces with Pi Camera to detect AprilTags  
│   ├── april_tag_detection.py    # Detects AprilTags and extracts their IDs  
│   ├── navigation.py             # Controls movement and navigation based on AprilTags  
│   ├── object_manipulation.py    # Controls robot actions (picking, placing objects)  
│   └── start_signal.py           # Detects match start LED or handles manual start switch  
│  
├── config/  
│   ├── hardware_config.yaml      # Hardware pin mappings and GPIO configurations  
│   └── april_tag_positions.yaml  # Known AprilTag positions on the field  
│  
├── tests/  
│   ├── test_camera_feed.py       # Verifies the camera setup and tag detection  
│   ├── test_navigation.py        # Simulates movement based on AprilTag input  
│   └── test_object_pickup.py     # Tests object manipulation logic  
│  
└── README.md                     # This file  
```

---

## Dependencies  
Make sure to install the following dependencies before running the code:  

```bash
sudo apt update  
sudo apt install python3 python3-pip  
pip3 install opencv-python numpy apriltag-python RPi.GPIO  
```

---

## System Components  

1. **Raspberry Pi 4**:  
   Acts as the main controller, handling camera input, logic processing, and motor control.  

2. **Pi Camera 2**:  
   Captures the field and detects AprilTags used for localization and navigation.  

3. **Motors and Actuators**:  
   Control the robot's movement and object manipulation mechanisms (e.g., grabbing Cosmic Shipping Containers).  

4. **Start LED Detection**:  
   The robot starts upon detecting the LED signal or via a manual switch as a fallback.  

---

## Code Overview  

- **`camera_control.py`**:  
   Initializes the Pi Camera, captures frames, and sends them to the AprilTag detection module.  

- **`april_tag_detection.py`**:  
   Uses OpenCV and the AprilTag library to detect tags and extract their IDs, helping the robot determine its location on the field.  

- **`navigation.py`**:  
   Controls the robot’s movement, using AprilTag detections to plan paths and reach key locations like the Cave or Rendezvous Pads.  

- **`object_manipulation.py`**:  
   Handles actions like picking up and placing Astral Materials or Shipping Containers based on the game rules.  

- **`start_signal.py`**:  
   Detects the match start LED signal and initiates autonomous execution. If the LED is not detected, it triggers the robot using a manual switch.  

---

## Workflow  

1. **Startup**:  
   - Robot waits for the **Start LED** signal.  
   - If the LED is not detected within a few seconds, the manual switch can be used to start the robot.  

2. **Navigation**:  
   - Robot navigates based on the AprilTags detected at key points.  
   - Known AprilTag IDs:
     - **5**: North Wall  
     - **6**: South Wall  
     - **7**: East Wall  
     - **0–4**: Telemetry Tag placed in the Beacon Mast indicating the preferred Rendezvous Pad.

3. **Object Manipulation**:  
   - The robot identifies **Cosmic Shipping Containers** and **Astral Materials** (Geodinium/Nebulite).  
   - Places the Astral Materials in appropriate containers to score points.  

4. **End of Match**:  
   - Robot ensures that key elements are in the correct locations to maximize post-match scoring.  

---

## How to Run the Code  

1. Connect the Pi Camera 2 and power up the Raspberry Pi 4.  
2. Navigate to the project directory.  
3. Run the main script:  

```bash
python3 src/main.py  
```  

4. Use the **tests/** directory to verify components individually. For example, to test the camera feed:  

```bash
python3 tests/test_camera_feed.py  
```  

---

## Troubleshooting  

- **Camera not detected**:  
   Ensure the Pi Camera is properly connected and enabled in the Raspberry Pi settings (`raspi-config`).  

- **AprilTag detection errors**:  
   Verify the camera is focused and the AprilTags are correctly oriented. Adjust lighting if necessary.  

- **Manual switch start not working**:  
   Check the GPIO pin configurations in `hardware_config.yaml` and ensure the switch is connected correctly.  

---

## Future Improvements  

- Implement **SLAM** for dynamic localization if AprilTags are blocked during gameplay.  
- Integrate additional sensors to improve precision in object manipulation.  
- Optimize the robot’s path planning to reduce match execution time.  

---

## References  
- [AprilTag Library](https://april.eecs.umich.edu/software/apriltag)  
- Raspberry Pi GPIO Documentation: https://gpiozero.readthedocs.io  

---

With this repository, our team aims to achieve autonomous excellence in the **Mining Mayhem** competition by efficiently reading AprilTags, manipulating objects, and completing all gameplay objectives. Let's bring home the win! 🎉  

---

