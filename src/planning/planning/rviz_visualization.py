import rclpy 
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np

class Markers(Node):
    def __init__(self):
        super().__init__("rviz_visualization")
        self.map_sub = self.create_subscription(OccupancyGrid, 'my_map', self.map_cb, 10)
        self.path_sub = self.create_subscription(Path, 'global_planner', self.path_cb, 10)
        self.publisher_ = self.create_publisher(MarkerArray, "marker_array", 10)
        self.timer_ = self.create_timer(1.0, self.marker_publisher)

        self.map_data = None
        self.obstacle_points = []
        self.path_poses = None
        self.start_point = None
        self.goal_point = None


    def map_cb(self, msg: OccupancyGrid):
        self.obstacle_points = []
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        for x in range(msg.info.width):
            for y in range(msg.info.height):
                if self.map_data[y, x] > 50:
                    point = Point()
                    x_m = x*msg.info.resolution + msg.info.origin.position.x + 0.025
                    y_m = y*msg.info.resolution + msg.info.origin.position.y + 0.025
                    point.x = x_m
                    point.y = y_m
                    point.z = 0.0
                    self.obstacle_points.append(point)


    def path_cb(self, msg: Path):
        self.path_poses = msg.poses   
        self.start_point = msg.poses[0].pose.position
        self.goal_point = msg.poses[-1].pose.position
        


    def marker_publisher(self):
        if self.start_point is None:
            return
        
        marker_array = MarkerArray()

        #shpere markers for start and goal points
        markers_attributes = {
            'start_marker' : [0, Marker.SPHERE, (self.start_point.x, self.start_point.y), (0.0, 1.0, 0.0)],
            'goal_marker' : [1, Marker.SPHERE, (self.goal_point.x, self.goal_point.y), (1.0,  0.0, 1.0)],
        }

        for marker_name in markers_attributes:
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.action = Marker.ADD
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = markers_attributes[marker_name][0]
            marker.type = markers_attributes[marker_name][1]
            marker.pose.position.x , marker.pose.position.y  = markers_attributes[marker_name][2]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0 
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.scale.z  = 0.05
            marker.color.a = 1.0
            marker.color.r, marker.color.g, marker.color.b = markers_attributes[marker_name][3]

            marker_array.markers.append(marker)
        
        #Line_strip marker for path
        path_points = []
        for pose in self.path_poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = 0.0
            path_points.append(point)

        path_marker = Marker()
        path_marker.header.frame_id = 'map'
        path_marker.action = Marker.ADD
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.id = 2
        path_marker.type =  Marker.LINE_STRIP
        path_marker.pose.position.z = 0.01
        path_marker.pose.orientation.w = 1.0 
        path_marker.scale.x = 0.01
        path_marker.color.a = 1.0
        path_marker.color.r, path_marker.color.g, path_marker.color.b = (0.0, 0.0, 1.0)
        path_marker.points = path_points

        marker_array.markers.append(path_marker)

        #cube_list marker for obstacles
        obstacle_marker = Marker()
        obstacle_marker.header.frame_id = 'map'
        obstacle_marker.action = Marker.ADD
        obstacle_marker.header.stamp = self.get_clock().now().to_msg()
        obstacle_marker.id = 3
        obstacle_marker.type = Marker.CUBE_LIST
        obstacle_marker.pose.position.z = 0.0
        obstacle_marker.pose.orientation.w = 1.0 
        obstacle_marker.scale.x = 0.05
        obstacle_marker.scale.y = 0.05
        obstacle_marker.scale.z  = 0.05
        obstacle_marker.color.a = 1.0
        obstacle_marker.color.r, obstacle_marker.color.g, obstacle_marker.color.b = (1.0, 0.0, 0.0)
        obstacle_marker.points = self.obstacle_points
        marker_array.markers.append(obstacle_marker)

        self.publisher_.publish(marker_array)


        
def main(args=None):
    rclpy.init(args=args)
    markers = Markers()
    try:
        rclpy.spin(markers)
    except KeyboardInterrupt:
        pass
    finally:
        markers.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()







