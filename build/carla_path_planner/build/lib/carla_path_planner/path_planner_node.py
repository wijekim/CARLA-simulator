#!/usr/bin/env python3
"""
CARLA Path Planner Node for ROS2 Nav2 Integration
Converts CARLA spawn points to ROS2 coordinate system and publishes Nav2 paths
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import carla
import math
from typing import List, Tuple, Optional


class CarlaPathPlanner(Node):
    """
    ROS2 Node that plans paths in CARLA using spawn points and publishes to /plan topic
    Uses nav_msgs/Path message type (standard ROS path message)
    """

    def __init__(self):
        super().__init__('carla_path_planner')
        
        # Declare parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('start_spawn_id', 0)
        self.declare_parameter('goal_spawn_id', 1)
        self.declare_parameter('waypoint_ids', '')  # 쉼표로 구분된 문자열
        self.declare_parameter('path_resolution', 1.0)  # meters between waypoints
        
        # Get parameters
        carla_host = self.get_parameter('carla_host').value
        carla_port = self.get_parameter('carla_port').value
        carla_timeout = self.get_parameter('carla_timeout').value
        
        # Connect to CARLA
        self.get_logger().info(f'Connecting to CARLA at {carla_host}:{carla_port}...')
        try:
            self.client = carla.Client(carla_host, carla_port)
            self.client.set_timeout(carla_timeout)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.get_logger().info(f'Connected to CARLA world: {self.map.name}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CARLA: {e}')
            raise
        
        # Initialize Global Route Planner
        try:
            from agents.navigation.global_route_planner import GlobalRoutePlanner
            path_resolution = self.get_parameter('path_resolution').value
            self.grp = GlobalRoutePlanner(self.map, path_resolution)
            self.get_logger().info(f'GlobalRoutePlanner initialized with resolution {path_resolution}m')
        except ImportError as e:
            self.get_logger().error(
                f'Failed to import GlobalRoutePlanner: {e}\n'
                'Make sure CARLA PythonAPI is in your PYTHONPATH:\n'
                'export PYTHONPATH=$PYTHONPATH:/path/to/CARLA_0.10.0/PythonAPI/carla'
            )
            raise
        
        # Create publisher for /plan topic
        self.path_publisher = self.create_publisher(Path, '/plan', 10)
        
        # Create timer to publish path periodically (1 Hz)
        self.timer = self.create_timer(1.0, self.publish_path)
        
        self.get_logger().info('CARLA Path Planner initialized')

    def carla_to_ros_transform(self, carla_location: carla.Location, 
                               carla_rotation: carla.Rotation) -> Tuple[float, float, float, float]:
        """
        Convert CARLA coordinates to ROS2 coordinate system
        
        CARLA: X-forward, Y-right, Z-up (left-handed)
        ROS2: X-forward, Y-left, Z-up (right-handed)
        
        Args:
            carla_location: CARLA Location object
            carla_rotation: CARLA Rotation object
            
        Returns:
            Tuple of (x, y, z, yaw) in ROS2 coordinate system
        """
        # Position transformation: Y-axis is inverted
        ros_x = carla_location.x
        ros_y = -carla_location.y
        ros_z = carla_location.z
        
        # Orientation transformation: Yaw is negated
        ros_yaw = -math.radians(carla_rotation.yaw)
        
        return ros_x, ros_y, ros_z, ros_yaw

    def euler_to_quaternion(self, yaw: float) -> Tuple[float, float, float, float]:
        """
        Convert yaw angle to quaternion
        
        Args:
            yaw: Yaw angle in radians
            
        Returns:
            Tuple of (x, y, z, w) quaternion components
        """
        qx = 0.0
        qy = 0.0
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)
        
        return qx, qy, qz, qw

    def get_spawn_point_transform(self, spawn_id: int) -> Optional[carla.Transform]:
        """
        Get CARLA spawn point transform by ID from the map's predefined spawn points
        
        CARLA 맵에는 미리 정의된 spawn point들이 있습니다.
        이 메서드는 그 중 하나를 ID로 가져옵니다.
        
        Args:
            spawn_id: Spawn point index (0부터 시작)
            
        Returns:
            CARLA Transform object or None if invalid ID
        """
        # CARLA 맵에 정의된 spawn point 가져오기
        spawn_points = self.map.get_spawn_points()
        
        if 0 <= spawn_id < len(spawn_points):
            self.get_logger().info(
                f'Using spawn point {spawn_id}: '
                f'x={spawn_points[spawn_id].location.x:.2f}, '
                f'y={spawn_points[spawn_id].location.y:.2f}'
            )
            return spawn_points[spawn_id]
        else:
            self.get_logger().error(
                f'Invalid spawn point ID: {spawn_id}. '
                f'Valid range: 0-{len(spawn_points)-1} '
                f'(Total {len(spawn_points)} spawn points available)'
            )
            return None

    def plan_route(self, start_id: int, goal_id: int, 
                   waypoint_ids: List[int] = None) -> Optional[List[carla.Waypoint]]:
        """
        Plan route using CARLA's road network
        
        Args:
            start_id: Starting spawn point ID
            goal_id: Goal spawn point ID
            waypoint_ids: List of intermediate waypoint spawn point IDs
            
        Returns:
            List of CARLA Waypoint objects forming the path
        """
        # Get spawn point transforms
        start_transform = self.get_spawn_point_transform(start_id)
        if start_transform is None:
            return None
        
        goal_transform = self.get_spawn_point_transform(goal_id)
        if goal_transform is None:
            return None
        
        # Get waypoints at spawn locations - only use Driving lanes
        start_waypoint = self.map.get_waypoint(
            start_transform.location, 
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        goal_waypoint = self.map.get_waypoint(
            goal_transform.location, 
            project_to_road=True,
            lane_type=carla.LaneType.Driving
        )
        
        if start_waypoint is None or goal_waypoint is None:
            self.get_logger().error('Failed to project spawn points to road')
            return None
        
        # 디버깅 정보 출력
        self.get_logger().info(
            f'Start waypoint: Lane {start_waypoint.lane_id}, Road {start_waypoint.road_id}, '
            f'Location ({start_waypoint.transform.location.x:.2f}, {start_waypoint.transform.location.y:.2f})'
        )
        self.get_logger().info(
            f'Goal waypoint: Lane {goal_waypoint.lane_id}, Road {goal_waypoint.road_id}, '
            f'Location ({goal_waypoint.transform.location.x:.2f}, {goal_waypoint.transform.location.y:.2f})'
        )
        
        # Build list of waypoints to visit
        waypoints_to_visit = [start_waypoint]
        
        # Add intermediate waypoints if provided
        if waypoint_ids:
            for wp_id in waypoint_ids:
                wp_transform = self.get_spawn_point_transform(wp_id)
                if wp_transform is not None:
                    wp = self.map.get_waypoint(
                        wp_transform.location, 
                        project_to_road=True,
                        lane_type=carla.LaneType.Driving
                    )
                    if wp is not None:
                        waypoints_to_visit.append(wp)
                        self.get_logger().info(
                            f'Waypoint {wp_id}: Lane {wp.lane_id}, Road {wp.road_id}'
                        )
        
        waypoints_to_visit.append(goal_waypoint)
        
        # Plan route between consecutive waypoints
        full_route = []
        resolution = self.get_parameter('path_resolution').value
        
        for i in range(len(waypoints_to_visit) - 1):
            current_wp = waypoints_to_visit[i]
            next_wp = waypoints_to_visit[i + 1]
            
            # Use CARLA's global route planner
            route = self.compute_route_waypoints(current_wp, next_wp, resolution)
            
            if route:
                # Avoid duplicate waypoints at segment boundaries
                if i > 0 and len(full_route) > 0:
                    route = route[1:]
                full_route.extend(route)
            else:
                self.get_logger().warn(
                    f'Failed to plan segment from waypoint {i} to {i+1}'
                )
        
        return full_route if full_route else None

    def compute_route_waypoints(self, start_wp: carla.Waypoint, 
                                end_wp: carla.Waypoint, 
                                resolution: float) -> List[carla.Waypoint]:
        """
        Compute route waypoints between two locations using A* on road network
        
        Args:
            start_wp: Starting waypoint
            end_wp: Goal waypoint
            resolution: Distance between waypoints in meters
            
        Returns:
            List of waypoints forming the route
        """
        try:
            # Plan route using initialized grp - returns list of (waypoint, RoadOption) tuples
            route = self.grp.trace_route(
                start_wp.transform.location,
                end_wp.transform.location
            )
            
            if not route:
                self.get_logger().warn(
                    f'No route found between waypoints at '
                    f'({start_wp.transform.location.x:.1f}, {start_wp.transform.location.y:.1f}) and '
                    f'({end_wp.transform.location.x:.1f}, {end_wp.transform.location.y:.1f})'
                )
                return []
            
            # Extract just the waypoints
            waypoints = [wp for wp, _ in route]
            
            self.get_logger().debug(f'Found route with {len(waypoints)} waypoints')
            
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f'Error planning route: {e}')
            return []

    def create_path_msg(self, waypoints: List[carla.Waypoint]) -> Path:
        """
        Create nav_msgs/Path message from CARLA waypoints
        
        Args:
            waypoints: List of CARLA Waypoint objects
            
        Returns:
            nav_msgs/Path message in ROS2 coordinate system
        """
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'
        
        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            
            # Convert CARLA coordinates to ROS2
            ros_x, ros_y, ros_z, ros_yaw = self.carla_to_ros_transform(
                waypoint.transform.location,
                waypoint.transform.rotation
            )
            
            # Set position
            pose_stamped.pose.position.x = ros_x
            pose_stamped.pose.position.y = ros_y
            pose_stamped.pose.position.z = ros_z
            
            # Set orientation (quaternion from yaw)
            qx, qy, qz, qw = self.euler_to_quaternion(ros_yaw)
            pose_stamped.pose.orientation.x = qx
            pose_stamped.pose.orientation.y = qy
            pose_stamped.pose.orientation.z = qz
            pose_stamped.pose.orientation.w = qw
            
            path_msg.poses.append(pose_stamped)
        
        return path_msg

    def publish_path(self):
        """
        Timer callback to plan and publish path
        """
        # Get parameters
        start_id = self.get_parameter('start_spawn_id').value
        goal_id = self.get_parameter('goal_spawn_id').value
        waypoint_ids_str = self.get_parameter('waypoint_ids').value
        
        # Parse waypoint_ids from comma-separated string
        waypoint_ids = []
        if waypoint_ids_str and waypoint_ids_str.strip():
            try:
                waypoint_ids = [int(x.strip()) for x in waypoint_ids_str.split(',')]
            except ValueError as e:
                self.get_logger().error(f'Invalid waypoint_ids format: {e}')
                return
        
        # Plan route
        if waypoint_ids:
            self.get_logger().info(
                f'Planning route from spawn {start_id} to spawn {goal_id} '
                f'via waypoints {waypoint_ids}'
            )
        else:
            self.get_logger().info(
                f'Planning route from spawn {start_id} to spawn {goal_id}'
            )
        
        route_waypoints = self.plan_route(start_id, goal_id, waypoint_ids)
        
        if route_waypoints is None or len(route_waypoints) == 0:
            self.get_logger().error('Failed to plan route')
            return
        
        # Create and publish path message
        path_msg = self.create_path_msg(route_waypoints)
        self.path_publisher.publish(path_msg)
        
        self.get_logger().info(
            f'Published path with {len(path_msg.poses)} waypoints'
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = CarlaPathPlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
