# 🧭 RTAB-MAP:(ROS 1 Noetic)

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
catkin build
```

Then source again:
```bash
source devel/setup.bash
```

## 🧱 Make the python script executable
```bash
cd /workspace/catkin_ws/scripts
chmod +x /workspace/catkin_ws/scripts/save_rtabmap_cloud.py
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
