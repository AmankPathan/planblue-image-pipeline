# ROS-Vision-Pipe

A ROS 2 (Humble) project implementing a simple distributed image processing pipeline using two nodes.

The system consists of:
- An image publisher node that continuously generates synthetic images
- An image subscriber node that overlays timestamps on images and saves both images and metadata to disk

---

## System Overview

### Image Publisher Node
- Publishes synthetic RGB images at a fixed rate
- Adds frame ID and ROS timestamp to each image message
- Logs publishing activity

### Image Subscriber Node
- Subscribes to the image stream
- Overlays timestamp text onto each image
- Saves images to disk
- Saves metadata (frame ID, filename, timestamp) to a JSON file
- Logs processing activity

On each run, the subscriber clears previous outputs to ensure clean results.

---

## Tech Stack

- ROS 2 Humble
- Python 3
- OpenCV
- NumPy
- cv_bridge

---

## Repository Structure
```text
ros-vision-pipe/
├── README.md
├── task_ws/
│   ├── src/
│   │   └── image_pipeline/
│   │       ├── image_pipeline/
│   │       │   ├── publisher_node.py
│   │       │   └── subscriber_node.py
│   │       ├── package.xml
│   │       └── setup.py
│   └── output/
│       ├── images/
│       └── metadata.json
```

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Python 3.10+
```bash
sudo apt install ros-humble-cv-bridge python3-opencv python3-colcon-common-extensions
```

---

## Installation
```bash
git clone <repository-url>
cd ros-vision-pipe
```

---

## Build Instructions
```bash
source /opt/ros/humble/setup.bash
cd task_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running the System

Start the subscriber first, then the publisher in a second terminal.

### Terminal 1 – Subscriber Node
```bash
source /opt/ros/humble/setup.bash
cd ros-vision-pipe/task_ws
source install/setup.bash
ros2 run image_pipeline image_subscriber
```

### Terminal 2 – Publisher Node
```bash
source /opt/ros/humble/setup.bash
cd ros-vision-pipe/task_ws
source install/setup.bash
ros2 run image_pipeline image_publisher
```

---

## Output

Output is saved to `task_ws/output/` and is reset on every subscriber startup.

### Images
- Location: `task_ws/output/images/`
- Format: PNG with timestamp overlay

### Metadata
- Location: `task_ws/output/metadata.json`
- Fields: `frame_id`, `filename`, `timestamp_unix`, `timestamp_human_utc`

---

## Notes

- Always start the subscriber before the publisher
- Output from previous runs is cleared on startup
- Both nodes log activity to the console