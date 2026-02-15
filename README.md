# PlanBlue Image Pipeline – ROS 2 Take-Home Task

This repository contains a ROS 2 (Humble) project implementing a simple distributed image processing pipeline using two ROS 2 nodes.

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
'''
---

## Installation

### ROS 2 Humble

Install ROS 2 Humble on Ubuntu 22.04 following the official ROS documentation.

Install required dependencies:

```bash
sudo apt install ros-humble-cv-bridge python3-opencv

