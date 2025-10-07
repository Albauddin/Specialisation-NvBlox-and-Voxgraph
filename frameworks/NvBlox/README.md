# 🧱 NvBlox: GPU-Accelerated 3D Mapping (ROS 2 Humble)

This directory contains the configuration, launch files, and analysis tools for running **NvBlox** — NVIDIA’s GPU-accelerated 3D reconstruction framework — on **ROS 2 Humble**.  
It enables **real-time 3D colored mapping** from RGB-D and IMU data, suitable for UAV-based Search & Rescue (SAR) applications.

---

## 📘 Installation

Follow the **official NVIDIA Isaac ROS NvBlox documentation** for setup and environment preparation:  
🔗 [https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html)

This includes:
  - Setting up the **Isaac ROS Humble Docker container**
  - Installing **NvBlox** and related dependencies
  - Verifying **RealSense D455** camera topics and tf frames

---

## 🐳 Run the Container

Once the setup is complete, start your NvBlox container:

``ˋbash
docker start isaac_ros_dev-x86_64-container
ˋˋˋ

run this to open the container in terminal
```bash
docker exec -it isaac_ros_dev-x86_64-container bash
ˋˋˋ

