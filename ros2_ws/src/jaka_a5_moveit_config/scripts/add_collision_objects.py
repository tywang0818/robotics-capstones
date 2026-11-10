#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


# Dimensions are [x, y, z] in metres. Poses are expressed in the world frame.
# Keep obstacle IDs stable so the same objects can be removed or replaced later.
OBSTACLES = [
    {
        "id": "table",
        "dimensions": [1.4, 1.4, 0.10],
        "position": [0.35, 0.0, -0.08],
    },
    {
        "id": "demo_box",
        "dimensions": [0.20, 0.20, 0.40],
        "position": [0.50, 0.35, 0.20],
    },
]


class CollisionObjectLoader(Node):
    def __init__(self):
        super().__init__("jaka_collision_object_loader")
        self.declare_parameter("remove", False)
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    @staticmethod
    def make_box(spec):
        collision_object = CollisionObject()
        collision_object.header.frame_id = "world"
        collision_object.id = spec["id"]

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = spec["dimensions"]

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = spec["position"]
        pose.orientation.w = 1.0

        collision_object.primitives.append(primitive)
        collision_object.primitive_poses.append(pose)
        collision_object.operation = CollisionObject.ADD
        return collision_object

    def make_request(self):
        remove = self.get_parameter("remove").value
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True

        for spec in OBSTACLES:
            if remove:
                collision_object = CollisionObject()
                collision_object.header.frame_id = "world"
                collision_object.id = spec["id"]
                collision_object.operation = CollisionObject.REMOVE
            else:
                collision_object = self.make_box(spec)
            request.scene.world.collision_objects.append(collision_object)

        return request, remove


def main(args=None):
    rclpy.init(args=args)
    node = CollisionObjectLoader()

    node.get_logger().info("Waiting for /apply_planning_scene...")
    if not node.client.wait_for_service(timeout_sec=15.0):
        node.get_logger().error(
            "/apply_planning_scene is unavailable. Start demo.launch.py first."
        )
        node.destroy_node()
        rclpy.shutdown()
        return

    request, remove = node.make_request()
    future = node.client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)

    if future.done() and future.result() is not None and future.result().success:
        action = "Removed" if remove else "Added"
        node.get_logger().info(f"{action} {len(OBSTACLES)} collision object(s).")
    else:
        node.get_logger().error("MoveIt did not accept the planning scene update.")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
