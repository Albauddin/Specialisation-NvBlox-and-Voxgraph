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

```bash
docker start isaac_ros_dev-x86_64-container
```

run this to open the container in terminal
```bash
docker exec -it isaac_ros_dev-x86_64-container bash
```

## 🎥 Recording a ROS Bag

You can record your own dataset following NVIDIA’s RealSense setup guide:
🔗 https://nvidia-isaac-ros.github.io/getting_started/hardware_setup/sensors/realsense_setup.html

To visualize and record using the RealSense Viewer tool:
```bash
realsense-viewer
```


## ▶️ Running Your ROS Bag

Visualize the pipeline:
```bash
rviz2 -d $(ros2 pkg prefix isaac_ros_visual_slam --share)/rviz/default.cfg.rviz
```

Launch NvBlox with your recorded ROS bag:
```bash
ros2 launch nvblox_examples_bringup realsense_example.launch.py rosbag:=<your rosbag path>
```

## 💾 Saving Meshes Automatically

While the stream or rosbag is running, you can save meshes automatically every 2 seconds.
Use either the Python script or the shell loop below.

Option 1 — Python Script
```bash
python3 periodic_mesh_saver.py
```
Option 2 — Bash Command
```bash
while true; do
  ros2 service call /nvblox_node/save_ply nvblox_msgs/srv/FilePath \
  "{file_path: '/workspaces/isaac_ros-dev/datasets/Meshes/mesh_$(date +%H%M%S).ply'}"
  sleep 2
done
```

This saves the reconstructed .ply mesh files every 2 seconds for later evaluation


## 📊 Analyzing Mesh Color Distribution

After mapping, analyze the color distribution and fidelity of the reconstructed meshes:
```bash
python3 batch_ply_color_hist.py
```

This script:

  - Processes all .ply meshes in a target directory
  - Extracts color histograms (R/G/B) for each mesh
  - Computes the average color metrics across all saved meshes






