# 🛰️ GPU-Accelerated 3D Colored Mapping for UAVs in Search & Rescue

In catastrophic situations, **rapid and accurate mapping** is essential to support mobile **Search & Rescue (SAR)** robots operating in challenging environments.  
This project focuses on **reconstructing 3D colored maps using GPU-accelerated frameworks** to provide rescue teams with **fast and detailed real-time situational awareness**.

While **LiDAR** offers precise point clouds, it is often **bulky, expensive, and lacks RGB or dense image information**.  
**Stereo-camera-based visual SLAM** presents a lightweight and cost-effective alternative that can be deployed efficiently on UAVs.

Two frameworks are evaluated in this study:
- **NvBlox (NVIDIA Isaac ROS)** — a GPU-accelerated mapping system for ROS 2  
- **Voxgraph (ETH Zürich)** — a submap-based TSDF SLAM framework for ROS 1  

Both frameworks are tested on an **NVIDIA Jetson Orin Nano 4 GB** to replicate real field conditions.  
A **common visual–inertial odometry source (OpenVINS)** provides localization to ensure a fair comparison.  
Evaluation focuses on the **quality and interpretability of reconstructed 3D colored meshes**, which are critical for decision-making in rescue operations.

The results demonstrate that **NvBlox produces more context-rich reconstructions**, enabling rescuers to interpret environments beyond simple geometry or color differences.  
Future extensions include optimizing **UAV scanning strategies** (e.g., trajectory design and flight speed) and **integrating object-recognition models** to enhance situational awareness and automatically identify potential survivors.

---

## 🧩 Repository Structure

This repository contains **two Docker-based environments**, each designed to replicate the experimental setup and enable reproducible testing:


├── NvBlox/ # GPU-accelerated mapping framework (ROS 2 Humble)
├── Voxgraph/ # Pose-graph-based mapping framework (ROS 1 Noetic + OpenVINS)
└── README.md # You are here



Each folder includes:
- Complete **Docker configuration** and build instructions  
- **Launch files** and **recording scripts** for RealSense D455  
- Detailed documentation of the experimental setup

---

## ⚙️ Hardware & Software Setup

- **Platform:** NVIDIA Jetson Orin Nano 4 GB  
- **Sensor:** Intel RealSense D455 (RGB-D + IMU)  
- **Localization:** OpenVINS (VIO)  
- **Frameworks:** NvBlox (ROS 2 Humble) and Voxgraph (ROS 1 Noetic)  
- **Environment:** Docker containers for reproducible builds and testing  

---

## 🔑 Keywords

*Search & Rescue (SAR), UAV Mapping, NvBlox, Voxgraph, OpenVINS, Jetson Orin Nano*
