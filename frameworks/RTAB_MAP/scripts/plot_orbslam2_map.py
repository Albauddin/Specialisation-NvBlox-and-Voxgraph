import numpy as np
import matplotlib.pyplot as plt

def load_xyz(path):
    # MapPoints.xyz typically: "x y z" per line (space-separated)
    return np.loadtxt(path)[:, :3]

def load_tum_trajectory(path):
    # TUM: timestamp tx ty tz qx qy qz qw
    data = np.loadtxt(path)
    return data[:, 1:4]  # tx ty tz

map_xyz = load_xyz("ORB_SLAM2_MapPoints.xyz")
traj_xyz = load_tum_trajectory("KeyFrameTrajectory_TUM_Format.txt")

fig = plt.figure()
ax = fig.add_subplot(111, projection="3d")

ax.scatter(map_xyz[:,0], map_xyz[:,1], map_xyz[:,2], s=0.5)
ax.plot(traj_xyz[:,0], traj_xyz[:,1], traj_xyz[:,2], linewidth=2)

ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.set_zlabel("Z")
plt.show()

