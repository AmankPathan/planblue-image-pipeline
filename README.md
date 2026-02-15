# PlanBlue Image Pipeline – ROS 2 Take-Home Task

This repository contains a ROS 2 (Humble) project implementing a simple distributed image processing pipeline using two nodes.

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

On each run, the subscriber clears previous outputs to ensure deterministic results.

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
planblue-image-pipeline/
├── README.md
├── planblue_task_ws/
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

## Installation

### ROS 2 Humble

Install ROS 2 Humble on Ubuntu 22.04 following the official ROS documentation.

Install required dependencies:

```bash
sudo apt install ros-humble-cv-bridge python3-opencv
```
## Clone the Repository
```bash
git clone <repository-url>
cd planblue-image-pipeline
```

## Build Instructions
```bash
source /opt/ros/humble/setup.bash
cd planblue_task_ws
colcon build
source install/setup.bash
```

## Running the System

Open two terminals.

### Terminal 1 – Subscriber Node
```bash
source /opt/ros/humble/setup.bash
cd planblue-image-pipeline/planblue_task_ws
source install/setup.bash
ros2 run image_pipeline image_subscriber
```

### Terminal 2 – Publisher Node
```bash
source /opt/ros/humble/setup.bash
cd planblue-image-pipeline/planblue_task_ws
source install/setup.bash
ros2 run image_pipeline image_publisher
```

## Output

After running the system, output files are saved to:

### Images
* Location: `planblue_task_ws/output/images/`
* Format: PNG images with timestamp overlay

### Metadata
* Location: `planblue_task_ws/output/metadata.json`
* Entries:
   * `frame_id`
   * `filename`
   * `timestamp`

## Notes
* Output directories are recreated on subscriber startup.
* Images and metadata from previous runs are overwritten.
* Both nodes log their activity to the console.