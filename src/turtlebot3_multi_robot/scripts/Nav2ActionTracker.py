#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from action_msgs.msg import GoalStatusArray, GoalStatus
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseWithCovarianceStamped

class Nav2ActionTracker(Node):
    def __init__(self):
        super().__init__('nav2_action_tracker')
        # List of robot namespaces
        self.robot_namespaces = ['/robot1', '/robot2', '/robot3', '/robot4', '/robot5']
        # Dictionary to track whether each robot’s nav2 action has succeeded.
        self.success_status = {ns: False for ns in self.robot_namespaces}
        # Latest known pose for robot1 (from, e.g., AMCL)
        self.robot1_pose = None
        # To prevent re-spawning the object repeatedly.
        self.object_spawned = False

        # Create a subscription for each robot’s nav2 action status topic.
        # These topics are part of the action protocol (status topic).
        self.status_subscribers = {}
        for ns in self.robot_namespaces:
            topic_name = ns + '/navigate_to_pose/_action/status'
            self.status_subscribers[ns] = self.create_subscription(
                GoalStatusArray,
                topic_name,
                lambda msg, ns=ns: self.status_callback(msg, ns),
                10
            )
            self.get_logger().info(f'Subscribed to {topic_name}')

        # Subscribe to robot1’s pose (for example, from the AMCL node).
        # Adjust the topic if your robot publishes its pose on another topic.
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/robot1/amcl_pose',
            self.robot1_pose_callback,
            10
        )

        # Publisher to spawn a custom object (using a Marker message)
        self.marker_pub = self.create_publisher(Marker, 'visualization_marker', 10)

        # Timer to periodically check if all robots have succeeded.
        self.check_timer = self.create_timer(1.0, self.check_all_success)

    def status_callback(self, msg: GoalStatusArray, ns: str):
        """
        Processes the incoming GoalStatusArray for a given robot.
        If any goal in the status list has succeeded, mark that robot as done.
        """
        # The action protocol defines 'SUCCEEDED' status as GoalStatus.STATUS_SUCCEEDED.
        # If any status in the array is successful, we mark that robot as succeeded.
        if any(status.status == GoalStatus.STATUS_SUCCEEDED for status in msg.status_list):
            if not self.success_status[ns]:
                self.get_logger().info(f'{ns} navigation succeeded.')
                self.success_status[ns] = True

    def robot1_pose_callback(self, msg: PoseWithCovarianceStamped):
        """
        Callback to capture robot1’s pose.
        """
        self.robot1_pose = msg.pose.pose

    def check_all_success(self):
        """
        Check if all robots have succeeded.
        If yes, spawn the custom object at robot1’s pose.
        """
        if all(self.success_status.values()):
            self.get_logger().info("All robots navigated successfully.")
            if self.robot1_pose is not None and not self.object_spawned:
                self.spawn_custom_object(self.robot1_pose)
                self.object_spawned = True
            # Once the custom object is spawned, stop checking.
            self.check_timer.cancel()

    def spawn_custom_object(self, pose):
        """
        Publishes a Marker message to visualize/spawn a custom object at the given pose.
        """
        marker = Marker()
        marker.header.frame_id = 'map'  # Adjust if using a different frame.
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'custom_object'
        marker.id = 0
        marker.type = Marker.CUBE  # Example: a cube. Change as needed.
        marker.action = Marker.ADD
        # Set the marker’s pose to robot1’s pose.
        marker.pose.position = pose.position
        marker.pose.orientation = pose.orientation
        # Define marker scale and color.
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0  # Fully opaque.
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)
        self.get_logger().info("Custom object spawned at robot1 pose.")

def main(args=None):
    rclpy.init(args=args)
    node = Nav2ActionTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
