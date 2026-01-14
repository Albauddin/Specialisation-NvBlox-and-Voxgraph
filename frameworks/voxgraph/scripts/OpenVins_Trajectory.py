#!/usr/bin/env python3
import os
import rospy
from nav_msgs.msg import Odometry

class OpenVinsTrajectoryLogger:
    def __init__(self):
        # Parameters
        self.topic = rospy.get_param("~topic", "/ov_msckf/odomimu")
        self.out_file = rospy.get_param("~out_file", "/workspace/catkin_ws/OpenVins_Trajectory_tum.txt")
        self.flush_every_n = int(rospy.get_param("~flush_every_n", 50))

        out_dir = os.path.dirname(self.out_file) or "."
        os.makedirs(out_dir, exist_ok=True)

        # 'w' overwrites each run. Use 'a' if you want to append.
        self.f = open(self.out_file, "w")
        self.count = 0

        rospy.loginfo("Subscribing to: %s", self.topic)
        rospy.loginfo("Logging TUM (t xyz qxyzw) to: %s", self.out_file)

        rospy.Subscriber(self.topic, Odometry, self.cb, queue_size=1000)

    def cb(self, msg):
        # Use message timestamp (rosbag-friendly)
        t = msg.header.stamp.to_sec()

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        # TUM format: timestamp tx ty tz qx qy qz qw
        self.f.write(
            f"{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} "
            f"{q.x:.8f} {q.y:.8f} {q.z:.8f} {q.w:.8f}\n"
        )

        self.count += 1
        if self.count % self.flush_every_n == 0:
            self.f.flush()

    def shutdown(self):
        try:
            self.f.flush()
            self.f.close()
        except Exception:
            pass

if __name__ == "__main__":
    rospy.init_node("openvins_trajectory_logger", anonymous=True)
    logger = OpenVinsTrajectoryLogger()
    rospy.on_shutdown(logger.shutdown)
    rospy.spin()

