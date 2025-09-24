import rclpy
from rclpy.node import Node
from nvblox_msgs.srv import FilePath
from datetime import datetime

class PeriodicMeshSaver(Node):
    def __init__(self):
        super().__init__('periodic_mesh_saver')
        self.cli = self.create_client(FilePath, '/nvblox_node/save_ply')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for save_ply service...')

        # Save every 3 seconds
        self.timer = self.create_timer(3.0, self.save_mesh)
        self.counter = 0

    def save_mesh(self):
        req = FilePath.Request()
        # Create a timestamped filename
        timestamp = datetime.now().strftime("%H%M%S")
        req.file_path = f"/workspaces/isaac_ros-dev/datasets/Meshes/mesh_{self.counter}_{timestamp}.ply"
        self.counter += 1

        future = self.cli.call_async(req)
        self.get_logger().info(f"Saving mesh to {req.file_path}")

def main(args=None):
    rclpy.init(args=args)
    node = PeriodicMeshSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

