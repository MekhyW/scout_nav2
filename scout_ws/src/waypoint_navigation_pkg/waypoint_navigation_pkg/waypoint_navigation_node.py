import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.callback_groups import ReentrantCallbackGroup
from std_srvs.srv import Trigger
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class WaypointNavigationNode(Node):
    def __init__(self):
        super().__init__('waypoint_navigation_node')
        self.callback_group = ReentrantCallbackGroup()
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self.callback_group)
        self.start_navigation_service = self.create_service(Trigger, 'start_navigation', self.start_navigation_callback, callback_group=self.callback_group)
        self.waypoints = self.load_waypoints()
        self.current_waypoint_index = 0
        self.navigation_active = False
        self.get_logger().info('Waypoint Navigation Node started')

    def load_waypoints(self):
        try:
            package_path = get_package_share_directory('waypoint_navigation_pkg')
            waypoints_file = os.path.join(package_path, 'waypoints.yaml')
            with open(waypoints_file, 'r') as f:
                waypoints_data = yaml.safe_load(f)
                return waypoints_data['waypoints']
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {str(e)}')
            return []

    async def start_navigation_callback(self, request, response):
        if not self.navigation_active:
            self.navigation_active = True
            self.current_waypoint_index = 0
            response.success = True
            response.message = 'Navigation started'
            self.get_logger().info('Starting navigation through waypoints')
            await self.navigate_to_next_waypoint()
        else:
            response.success = False
            response.message = 'Navigation already in progress'
        return response

    async def navigate_to_next_waypoint(self):
        if not self.navigation_active or self.current_waypoint_index >= len(self.waypoints):
            self.navigation_active = False
            self.get_logger().info('Navigation completed')
            return
        waypoint = self.waypoints[self.current_waypoint_index]
        self.get_logger().info(f'Navigating to waypoint: {waypoint["name"]}')
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = waypoint['pose']['position']['x']
        goal_msg.pose.pose.position.y = waypoint['pose']['position']['y']
        goal_msg.pose.pose.position.z = waypoint['pose']['position']['z']
        goal_msg.pose.pose.orientation.x = waypoint['pose']['orientation']['x']
        goal_msg.pose.pose.orientation.y = waypoint['pose']['orientation']['y']
        goal_msg.pose.pose.orientation.z = waypoint['pose']['orientation']['z']
        goal_msg.pose.pose.orientation.w = waypoint['pose']['orientation']['w']
        while not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for navigation action server...')
        goal_handle = await self.nav_client.send_goal_async(goal_msg)
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        result = await goal_handle.get_result_async()
        if result.result.result:
            self.get_logger().info(f'Reached waypoint {waypoint["name"]}')
            self.current_waypoint_index += 1
            await self.navigate_to_next_waypoint()
        else:
            self.get_logger().error('Navigation failed')
            self.navigation_active = False

def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()