# 🧭 RTAB-MAP:(ROS 1 Noetic)

For this project, RTAB-Map is used via the ROS wrapper provided in the repository
https://github.com/laukik-hase/rsslam_ws/tree/master/src/rtabmap_ros
,
which contains a ROS 1–compatible integration of the RTAB-Map framework.

RTAB-Map (Real-Time Appearance-Based Mapping) is a graph-based SLAM system designed for long-term and large-scale mapping. It performs simultaneous localization and mapping using visual or RGB-D sensor data, with optional support for additional inputs such as IMU or wheel odometry. The system incrementally builds a pose graph in which nodes represent robot poses and edges encode spatial constraints derived from visual odometry and loop closures.

Key characteristics of RTAB-Map include:

Appearance-based loop closure detection using visual feature descriptors,

Graph optimization for global pose consistency,

Support for RGB-D cameras, including Intel RealSense sensors,

Generation of dense point clouds and 3D maps from accumulated depth data.

In this project, RTAB-Map is applied to the same RealSense-based datasets as the other SLAM frameworks to ensure comparability. Its outputs—camera trajectories, pose graphs, and reconstructed point clouds—are used for quantitative and qualitative comparison against feature-based and TSDF-based mapping approaches.

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
docker run -it --name RTAB_MAP --net=host --privileged\
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
2️⃣ set simulation time and run the rosbag
```bash
rosparam set use_sim_time true 
rosbag play  /workspace/catkin_ws/dataset/bags/scene_raw.bag --clock --pause -r 1.0 
```
3️⃣ run the ORB-SLAM2 Stereo
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

