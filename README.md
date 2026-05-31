<div align="center">

# 🚗 Sensor Fusion — 3D Object Tracking & Time-to-Collision Estimation

### Multi-Sensor Perception Using Camera and LiDAR Data

![C++](https://img.shields.io/badge/C++-Sensor_Fusion-blue?style=for-the-badge&logo=cplusplus)
![OpenCV](https://img.shields.io/badge/OpenCV-Computer_Vision-green?style=for-the-badge&logo=opencv)
![LiDAR](https://img.shields.io/badge/LiDAR-3D_Perception-orange?style=for-the-badge)
![Autonomous Driving](https://img.shields.io/badge/Autonomous_Driving-Object_Tracking-red?style=for-the-badge)
![Research](https://img.shields.io/badge/Experimental_Benchmarking-Detector_Comparison-purple?style=for-the-badge)

Udacity Sensor Fusion Nanodegree Final Project

</div>

---

# Overview

This project combines camera and LiDAR measurements to track objects in 3D space and estimate **Time-to-Collision (TTC)** between the ego vehicle and the vehicle ahead.

The system performs:

- Object detection
- Object association across frames
- Keypoint matching
- Camera-based TTC estimation
- LiDAR-based TTC estimation
- Detector/descriptor benchmarking

The project serves as a complete perception pipeline and represents the culmination of the Sensor Fusion Camera course.

---

## System Architecture

<p align="center">
  <img src="images/course_code_structure.png" width="900">
</p>

---

# Project Objectives

The goals of this project were to:

- Track objects through time
- Associate LiDAR and camera observations
- Estimate TTC using LiDAR
- Estimate TTC using camera imagery
- Compare detector/descriptor combinations
- Evaluate perception reliability

---

# Why Time-to-Collision Matters

Time-to-Collision is one of the most important measurements in autonomous driving.

It answers a simple question:

> If current motion continues, how long until the ego vehicle reaches the object ahead?

Reliable TTC estimation supports:

- Forward collision warning
- Emergency braking
- Adaptive cruise control
- Risk assessment systems

---

# Processing Pipeline

```text
Camera Images
        ↓
Feature Detection
        ↓
Feature Description
        ↓
Feature Matching
        ↓
Object Association
        ↓
Camera TTC Estimation


LiDAR Point Clouds
        ↓
Object Association
        ↓
Distance Estimation
        ↓
LiDAR TTC Estimation


Camera TTC + LiDAR TTC
            ↓
Performance Evaluation
```

---

# 3D Object Tracking

The first challenge is associating detected objects across successive frames.

The system uses:

- Bounding boxes
- Keypoint correspondences
- Match statistics

to identify which object in the current frame corresponds to which object in the previous frame. :contentReference[oaicite:1]{index=1}

This creates a consistent object identity through time.

---

# LiDAR-Based TTC

LiDAR provides direct distance measurements.

The workflow:

1. Crop points to the ego lane
2. Remove outliers
3. Compute object distance
4. Estimate TTC from distance changes across frames

Because LiDAR measures physical distance directly, TTC estimates are highly stable and consistent. :contentReference[oaicite:2]{index=2}

---

# Camera-Based TTC

Camera TTC estimation relies on image geometry.

The workflow:

1. Detect keypoints
2. Match keypoints between frames
3. Measure scale changes
4. Estimate TTC from keypoint expansion

As the vehicle ahead approaches, image features grow larger.

The rate of growth provides an estimate of Time-to-Collision. :contentReference[oaicite:3]{index=3}

---

# Detector and Descriptor Evaluation

One of the most interesting aspects of this project is the systematic comparison of detector and descriptor combinations.

## Detectors Tested

- HARRIS
- SHITOMASI
- FAST
- BRISK
- ORB
- AKAZE
- SIFT

## Descriptors Tested

- BRISK
- BRIEF
- ORB
- FREAK
- AKAZE
- SIFT

All valid combinations were tested automatically using a custom benchmarking workflow. :contentReference[oaicite:4]{index=4}

---

# Automation Framework

To evaluate detector performance efficiently:

- The C++ application was modified to accept command-line parameters
- Automated scripts executed all valid combinations
- Results were stored as CSV files
- Python analysis scripts generated performance summaries

This transformed the project from a single experiment into a reproducible benchmarking framework. 

---

# Results Summary

## Best Overall Combination

### SIFT + BRISK

Advantages:

- Stable camera TTC
- Strong matching performance
- Results closest to LiDAR estimates

---

## Fastest Reliable Combination

### FAST + BRISK

Advantages:

- Very fast execution
- Good TTC estimation
- Suitable for real-time applications

---

## LiDAR Performance

LiDAR TTC remained remarkably stable across detector configurations:

```text
Mean TTC ≈ 11–12 seconds
```

LiDAR measurements were less sensitive to environmental conditions and feature matching errors. :contentReference[oaicite:6]{index=6}

---

# Technical Skills Demonstrated

## Sensor Fusion

- Camera + LiDAR Integration
- Multi-Sensor Perception
- Object Association

## Computer Vision

- Feature Detection
- Feature Description
- Feature Matching
- Object Tracking

## Autonomous Systems

- TTC Estimation
- Collision Risk Analysis
- Environmental Understanding

## Software Engineering

- Modern C++
- OpenCV
- Benchmark Automation
- Experimental Analysis

---

# Repository Structure

```text
src/
├── camFusion.cpp
├── matching2D.cpp
├── lidarData.cpp
├── objectTracking.cpp

images/
├── course_code_structure.png

results/
├── TTC evaluations
├── detector benchmarks

README.md
```

---

# Key Concepts Explored

- Sensor Fusion
- Time-to-Collision
- LiDAR
- Computer Vision
- Object Tracking
- Keypoint Matching
- Feature Descriptors
- Perception Systems
- Autonomous Driving

---

# Why This Project Matters

Modern autonomous vehicles rarely rely on a single sensor.

Each sensor has strengths and weaknesses:

| Sensor | Strength |
|----------|----------|
| Camera | Rich visual information |
| LiDAR | Accurate distance measurements |

Combining both sensors provides a more robust perception system than either sensor alone.

This project demonstrates how multiple sensing modalities can be used together to estimate collision risk in real-world driving scenarios.

---

# Related Sensor Fusion Projects

This repository is part of a broader autonomous systems portfolio including:

- Camera-Based 2D Feature Tracking
- LiDAR Obstacle Detection
- Extended Kalman Filter Sensor Fusion
- Kidnapped Vehicle Localization
- Highway Path Planning
- PID Control

Together these projects cover perception, localization, tracking, planning, and control for autonomous systems.

---

# Learning Outcomes

This project provided practical experience with:

- Multi-sensor perception
- Object tracking
- TTC estimation
- Detector benchmarking
- Automated experimentation
- Camera and LiDAR integration

It represents a realistic example of perception engineering for autonomous systems.

---

# Build Instructions

## Dependencies

- CMake
- OpenCV
- GCC / G++
- Modern C++ Compiler

## Build

```bash
mkdir build
cd build

cmake ..
make
```

Run:

```bash
./3D_object_tracking
```

---

# Disclaimer

This repository is provided for educational and portfolio purposes.

Students may study the code and reports for learning purposes, but submitting this work as coursework would constitute plagiarism and may violate academic integrity policies.

Copyright © Sabrina Palis

