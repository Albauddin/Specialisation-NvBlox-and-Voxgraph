# 🧭 ORB-SLAM2:(ROS 1 Noetic)

For this project, we use the repository
https://github.com/NERanger/ORB-SLAM2-with-D435i
,
which is a RealSense-adapted fork of ORB-SLAM2 with native support for Intel RealSense stereo and RGB-D cameras.

This repository was selected because it provides:

Direct integration with RealSense D435i/D455 (stereo IR + optional IMU support),

A ready-to-use ROS 1 Noetic wrapper, simplifying deployment on recorded rosbags and live camera streams,

Minimal modification of the original ORB-SLAM2 core, preserving algorithmic correctness while improving sensor compatibility.

ORB-SLAM2 is a feature-based visual SLAM system that estimates the camera trajectory and reconstructs a sparse 3D map of the environment. It relies on:

ORB feature extraction and matching,

Keyframe-based tracking and local mapping,

Graph-based pose optimization with loop closure detection for global consistency.

---

## 📦 Build the Container

From your workspace, build the base Docker image:if you already done the Voxgraph method this part is not necessary 

```bash
cd /<YOUR_WORKSPACE_PATH>/workspaces/Voxgraph/docker
docker build -f Dockerfile.base -t voxgraph-dev:base .
```


## 🐳 Run the Container

Start the container with full device and display access:
```bash
docker run -it --rm --name orb_slam2_dev_2nd --net=host --privileged\
  -e DISPLAY
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  -v /dev:/dev
  -v /<YOUR_WORKSPACE_PATH>:/workspace/catkin_ws
```

Then source your workspace:
```bash
source /workspace/catkin_ws/devel/setup.bash
```

## 📚 clone the repositoriy

Clone the required packages inside the container:
(https://github.com/NERanger/ORB-SLAM2-with-D435i)
```bash
git clone https://github.com/NERanger/ORB-SLAM2-with-D435i.git
```

## Here you can configure some settings if necessary (DON'T CHANGE THIS FOR EAR_LAB)

For orb slam in Ros_stereo.cc add the function SaveCloudMap
and the file map_cloud.ply will besaved


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


## 🧩 Offline Mapping from ROS Bag

Launch the following components in separate terminals inside the container.

1️⃣ run the ros core (Terminal 1)
```bash
roscore 
```
2️⃣ set simulation time and run the rosbag (Terminal 2)
```bash
rosparam set use_sim_time true 
rosbag play  /workspace/catkin_ws/dataset/bags/scene_raw.bag --clock --pause -r 1.0 
```
3️⃣ run the ORB-SLAM2 Stereo (Terminal 3)
```bash
rosrun ORB_SLAM2 Stereo \ 
  /workspace/catkin_ws/src/ORB_SLAM2_NOETIC/Vocabulary/ORBvoc.txt \ 
  /workspace/catkin_ws/src/ORB_SLAM2_NOETIC/Examples/ROS/ORB_SLAM2/Realsense_AsusStyle_848x480.yaml \ 
  false \ 
  /camera/left/image_raw:=/camera/infra1/image_rect_raw \ 
  /camera/right/image_raw:=/camera/infra2/image_rect_raw 
```


## 🎥 Record a ROS Bag

assuming you already have the recorede rosbag from Voxgraph framework


## 🗺️ Visualization

Meshes can be viewed using **CloudCompare** for visual inspection and quality evaluation.

Example command to launch CloudCompare (on host):

```bash
flatpak run org.cloudcompare.CloudCompare
```
