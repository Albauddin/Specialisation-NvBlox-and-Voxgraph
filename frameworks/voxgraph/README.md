# 🧭 Voxgraph: Submap-Based TSDF Mapping (ROS 1 Noetic)

This directory provides the configuration and instructions for running **Voxgraph**, a submap-based 3D reconstruction framework from **ETH Zürich**, using stereo RGB-D input and **OpenVINS** visual–inertial odometry.  
The containerized setup enables reproducible mapping experiments on UAV datasets with the **Intel RealSense D455** and **NVIDIA Jetson Orin Nano**.

---

## 📦 Build the Container

From your workspace, build the base Docker image:

```bash
cd /<YOUR_WORKSPACE_PATH>/workspaces/Voxgraph/docker
docker build -f Dockerfile.base -t voxgraph-dev:base .
```


## 🐳 Run the Container

Start the container with full device and display access:
```bash
docker run -it --rm --name vox_dev --net=host --privileged \
  -e DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v /dev:/dev \
  -v /<YOUR_WORKSPACE_PATH>/workspaces/Voxgraph:/workspace/catkin_ws \
  voxgraph-dev:base
```

Then source your workspace:
```bash
source /workspace/catkin_ws/devel/setup.bash
```

## 📚 Fetch Dependencies

Clone the required packages inside the container:
```bash
cd /workspace/catkin_ws/src

git clone https://github.com/catkin/catkin_simple.git 
git clone https://github.com/ethz-asl/eigen_catkin.git 
git clone https://github.com/ethz-asl/glog_catkin.git 
git clone https://github.com/ethz-asl/gflags_catkin.git 
git clone https://github.com/ethz-asl/eigen_checks.git 
git clone https://github.com/ethz-asl/minkindr.git 
git clone https://github.com/ethz-asl/minkindr_ros.git 
git clone https://github.com/ethz-asl/kindr.git -b catkin 
git clone https://github.com/ethz-asl/catkin_boost_python_buildtool.git 
git clone https://github.com/ethz-asl/numpy_eigen.git 
git clone https://github.com/ethz-asl/protobuf_catkin.git 
git clone https://github.com/pal-robotics/ddynamic_reconfigure.git 
git clone https://github.com/ethz-asl/cblox.git   
git clone https://github.com/ethz-asl/ceres_catkin.git 
git clone https://github.com/ethz-asl/gtsam_catkin.git
```

## 🧱 Build the Workspace

After cloning all dependencies, build everything:
```bash
cd /workspace/catkin_ws
catkin build
```

Then source again:
```bash
source devel/setup.bash
```


## 🚀 Running in Real-Time (Live Input)

Launch the following components in separate terminals inside the container.

1️⃣ RealSense Camera
```bash
roslaunch realsense2_camera rs_camera.launch \
  enable_depth:=true enable_color:=true \
  depth_module.profile:=848x480x30 \
  color_width:=848 color_height:=480 color_fps:=30 \
  enable_pointcloud:=true align_depth:=true \
  enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true \
  unite_imu_method:=linear_interpolation \
  initial_reset:=true queue_size:=1 publish_tf:=false
```
2️⃣ OpenVINS Localization
```bash
roslaunch /workspace/catkin_ws/launch/ov_realsense.launch --screen
```
3️⃣ Voxgraph Mapping
```bash
roslaunch voxgraph voxgraph_realtime.launch show_rviz:=true
```
  ⚠️ Note: Real-time performance on Jetson Orin Nano may vary. For best results, record a rosbag and replay it at a slower rate (0.5–0.1× speed).


## 🎥 Record a ROS Bag

To record data for offline mapping:

1️⃣ Launch RealSense
```bash
roslaunch realsense2_camera rs_camera.launch \
  enable_depth:=true enable_color:=true \
  depth_module.profile:=848x480x30 \
  color_width:=848 color_height:=480 color_fps:=30 \
  enable_pointcloud:=true align_depth:=true \
  enable_infra1:=true enable_infra2:=true \
  enable_gyro:=true enable_accel:=true \
  unite_imu_method:=linear_interpolation \
  initial_reset:=true queue_size:=1 publish_tf:=true
```
2️⃣ Record Bag with Required Topics
```bash
rosbag record -O scene_raw.bag -b 4096 \
  /camera/infra1/image_rect_raw \
  /camera/infra1/camera_info \
  /camera/infra2/image_rect_raw \
  /camera/infra2/camera_info \
  /camera/imu \
  /camera/depth/color/points \
  /camera/color/image_raw \
  /camera/color/camera_info \
  /tf \
  /tf_static
```

## 🧩 Offline Mapping from ROS Bag
Terminal 1 – Core
```bash
roscore
```
Terminal 2 – OpenVINS Localization
```bash
rosparam set use_sim_time true
roslaunch /workspace/catkin_ws/launch/ov_realsense.launch use_sim_time:=true
```
Terminal 3 – Voxgraph Mapping
```bash
roslaunch voxgraph voxgraph_realtime.launch show_rviz:=true use_sim_time:=true
```
Terminal 4 – Play Rosbag
```bash
rosbag play scene_raw.bag --clock --rate 0.5
```


## 🗺️ Visualization

Meshes can be viewed using **CloudCompare** for visual inspection and quality evaluation.

Example command to launch CloudCompare (on host):

```bash
flatpak run org.cloudcompare.CloudCompare
```


# 🗺️ Mapping Results from This Project

---

## 🛰️ Localization from OpenVINS
<img width="100%" alt="OpenVINS Localization" src="https://github.com/user-attachments/assets/237fc447-7fd3-45e7-99d1-29f2cd9f230a" />

---

## 🧩 Complete Map
<img width="100%" alt="Complete Map" src="https://github.com/user-attachments/assets/1cfaff92-ad36-4567-8583-79330e7b297a" />

---

## 🕸️ Pose Graph Optimization and Submaps
<p align="center">
  <img width="48%" alt="Pose Graph 1" src="https://github.com/user-attachments/assets/01de524a-32e6-4e65-abfb-7ae3992de1f5" />
  <img width="48%" alt="Pose Graph 2" src="https://github.com/user-attachments/assets/67ed65bf-bca8-48ef-ba52-ce301a3348bf" />
</p>

---

## 🚗 Car Scene
<p align="center">
  <img width="48%" alt="Car Scene 1" src="https://github.com/user-attachments/assets/8ac017a9-0dc1-4c1b-a342-f0498d9eea6b" />
  <img width="48%" alt="Car Scene 2" src="https://github.com/user-attachments/assets/d772a4e3-1186-4727-950b-866de923fb54" />
</p>

---

## 🪟 Window
<img width="90%" alt="Window" src="https://github.com/user-attachments/assets/2fa5c813-bfc5-4490-86ae-3f69243e9ae6" />

---

## 🌿 Plants
<p align="center">
  <img width="48%" alt="Plants 1" src="https://github.com/user-attachments/assets/e245ce1f-54d9-462a-85d2-2f79bbcc2695" />
  <img width="48%" alt="Plants 2" src="https://github.com/user-attachments/assets/d36ff257-3c3f-46d9-9fa8-3ce5962cd82f" />
</p>

---

## 🚪 Garage
<img width="90%" alt="Garage" src="https://github.com/user-attachments/assets/93a112bd-2110-4585-8960-40914b4c1c56" />

---

## 🗑️ Trash Bin
<img width="85%" alt="Trash Bin" src="https://github.com/user-attachments/assets/d507a229-3873-4a77-a382-7b9ad04bc5ce" />

---

## 💡 Wall Lamp Detail
<p align="center">
  <img width="48%" alt="Wall Lamp 1" src="https://github.com/user-attachments/assets/cfc675ac-eeed-4119-ba4d-4cb1975aad22" />
  <img width="48%" alt="Wall Lamp 2" src="https://github.com/user-attachments/assets/b76d1261-2a6c-4e57-be2c-54df8b92ea6f" />
</p>















