#!/usr/bin/env python3

import rospy
from rtabmap_msgs.srv import ExportMap, ExportMapRequest
import os
import time

def save_rtabmap_cloud():
    rospy.init_node("rtabmap_map_saver", anonymous=True)

    # Container-internal export directory
    export_dir = "/workspace/catkin_ws/export"
    os.makedirs(export_dir, exist_ok=True)

    cloud_path = os.path.join(export_dir, "map_cloud.ply")

    rospy.loginfo("Waiting for RTAB-Map export_map service...")
    rospy.wait_for_service("/rtabmap/export_map")

    try:
        export_map = rospy.ServiceProxy("/rtabmap/export_map", ExportMap)

        req = ExportMapRequest()
        req.cloud_output = cloud_path
        req.binary = True
        req.optimize = True
        req.assemble = True
        req.voxel_size = 0.0
        req.mesh = False
        req.normal_k = 0

        rospy.loginfo("Exporting RTAB-Map point cloud...")
        export_map(req)

        rospy.loginfo(f"RTAB-Map cloud successfully saved to: {cloud_path}")

    except rospy.ServiceException as e:
        rospy.logerr(f"RTAB-Map export failed: {e}")

if __name__ == "__main__":
    # Ensure RTAB-Map has fully finished processing
    time.sleep(2.0)
    save_rtabmap_cloud()

