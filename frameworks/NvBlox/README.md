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

## 🧭 Install OpenVINS for ROS 2

For visual–inertial localization, install OpenVINS (ROS 2 version) following the official documentation:

🔗 https://docs.openvins.com/gs-installing.html

This package provides high-accuracy visual–inertial odometry that NvBlox can use as a localization source during reconstruction.

## 🔁 Remapping Topics for OpenVINS and NvBlox

After installing OpenVINS, make sure that **the topic names and coordinate frames** used by OpenVINS match those expected by NvBlox.

You can adjust these mappings directly in the **NvBlox YAML configuration file** (or in your custom launch file) to ensure the correct data flow between realsense, OpenVINS and NvBlox.







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
ros2 launch nvblox_examples_bringup realsense_example.launch.py rosbag:=<YOUR_ACTUAL_ROSBAG_PATH>
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
  "{file_path: '/<YOUR_ACTUAL_MESHES_PATH>s/mesh_$(date +%H%M%S).ply'}"
  sleep 2
done
```

This saves the reconstructed .ply mesh files every 2 seconds for later evaluation


## 📊 Analyzing Mesh Color Distribution

After mapping, analyze the color distribution and fidelity of the reconstructed meshes:
```bash
python3 batch_ply_color_hist.py /<YOUR_MESHES_DIRECTORY_PATH>
```

This script:

  - Processes all .ply meshes in a target directory
  - Extracts color histograms (R/G/B) for each mesh
  - Computes the average color metrics across all saved meshes


## 🗺️ Viewing the Meshes in CloudCompare

Download and install CloudCompare to visualize and inspect the exported meshes:
🔗 https://cloudcompare.org/index.html

To launch CloudCompare:
```bash
flatpak run org.cloudcompare.CloudCompare
```

Then import the generated .ply meshes to analyze the 3D reconstruction results visually.


# 🗺️ Mapping Results from This Project

## 🧩 Complete Map
<img width="100%" alt="GetImage (9)" src="https://github.com/user-attachments/assets/e74a1341-d6fa-480a-b964-d89bbfa32a0a" />

---

## 🚗 Car Scene
<p align="center">
  <img width="48%" alt="Car Scene 1" src="https://github.com/user-attachments/assets/77a1cbbe-a1ed-4d86-9822-7dda8fe4e6ad" />
  <img width="48%" alt="Car Scene 2" src="https://github.com/user-attachments/assets/375c8443-8bda-4a34-8aa9-5dadd945092a" />
</p>

---

## 🪟 Window
<img width="90%" alt="GetImage (3)" src="https://github.com/user-attachments/assets/d52ba27d-9a21-442d-b353-21ae884d4f40" />

---

## 🌿 Plants
<p align="center">
  <img width="48%" alt="Plants 1" src="https://github.com/user-attachments/assets/46eb5fac-ffd5-4cbc-816c-c399f09722b8" />
  <img width="48%" alt="Plants 2" src="https://github.com/user-attachments/assets/03534b32-b25e-4820-8379-283dd0bb7dbd" />
</p>

---

## 🚪 Garage
<img width="90%" alt="GetImage (5)" src="https://github.com/user-attachments/assets/711967e6-b0bc-4445-932b-10821ba04e16" />

---

## 🗑️ Trash Bin
<img width="80%" alt="GetImage (6)" src="https://github.com/user-attachments/assets/24edd88d-d276-4598-ab7f-f3958e0fbe0d" />

---

## ⚠️ Misalignment
<img width="85%" alt="GetImage (7)" src="https://github.com/user-attachments/assets/426a324a-aa4c-4be1-ab55-4d0e6d93312a" />

---

## 🌬️ Ventilation Detail
<img width="90%" alt="GetImage (8)" src="https://github.com/user-attachments/assets/ed28a342-e1be-4f6a-a8ee-6d96675d3a92" />

---

## 💡 Wall Lamp Detail
<p align="center">
  <img width="48%" alt="Wall Lamp 1" src="https://github.com/user-attachments/assets/13b73010-1837-4f38-9d69-4564d85a4289" />
  <img width="48%" alt="Wall Lamp 2" src="https://github.com/user-attachments/assets/b93b6de7-748e-48dd-bd24-3c3d0353c6ed" />
</p>




















