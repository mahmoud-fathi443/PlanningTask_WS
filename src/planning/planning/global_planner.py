import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import heapq


class Planner(Node):
    def __init__(self):
        super().__init__("global_planner")

        self.path_pub = self.create_publisher(Path, "global_planner", 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.map_sub = self.create_subscription(OccupancyGrid, 'my_map', self.map_callback, 10)
        self.planner_timer = self.create_timer(0.1, self.planner_callback)

        self.map_data = None
        self.map_width = None
        self.map_height = None
        self.map_resolution = None
        self.map_origin_x = None
        self.map_origin_y = None
        self.current_x = None
        self.current_y = None
        self.start_x, self.start_y = 0.0, 0.0
        self.goal_x , self.goal_y = 1.5, 1.0
        self.orthogonal_step_cost = 1.0
        self.diagonal_step_cost = self.orthogonal_step_cost * 1.4142


    def map_callback(self, msg: OccupancyGrid):
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_origin_x = msg.info.origin.position.x
        self.map_origin_y = msg.info.origin.position.y
        self.map_resolution = msg.info.resolution
        self.map_data = np.array(msg.data).reshape((self.map_height, self.map_width))

    def planner_callback(self):
        #publish path
        if self.map_data is None:
            self.get_logger().warn("Map not received!")
            return
        
        self.current_x, self.current_y = self.meters_to_grid(self.start_x, self.start_y)
        goal_x, goal_y = self.meters_to_grid(self.goal_x, self.goal_y)

        path = self.A_Algorithm(self.current_x, self.current_y, goal_x, goal_y)
        if path:
            self.publish_path(path)
            self.get_logger().info("path published")
        else:
            self.get_logger().warn("Path not found!")
        
        #broadcast map -> planner_frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = "planner_frame"
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


        


    def meters_to_grid(self, x_m, y_m):
        x_grid = int((x_m - self.map_origin_x) // self.map_resolution)
        y_grid = int((y_m - self.map_origin_y) // self.map_resolution)

        return x_grid, y_grid
    
    def grid_to_meters(self, x_g, y_g):
        x_m = (x_g * self.map_resolution) + self.map_origin_x
        y_m = (y_g * self.map_resolution) + self.map_origin_y


        return x_m, y_m
    
    def h(self, x, y, goal_x, goal_y):
        return np.sqrt((goal_x-x)**2+(goal_y-y)**2)
    
    def valid_nodes(self, current):
        
        neighbours = []
        LETHAL_COST = 50
        current_x, current_y = current

        valid_moves = [
            (1, 0, self.orthogonal_step_cost),  # right
            (-1, 0, self.orthogonal_step_cost), # left
            (0, 1, self.orthogonal_step_cost),  # up
            (0, -1, self.orthogonal_step_cost), # down
            (1, 1, self.diagonal_step_cost),  # top-right
            (-1, 1, self.diagonal_step_cost), # top-left
            (1, -1, self.diagonal_step_cost), # bottom-right
            (-1, -1, self.diagonal_step_cost),# bottom-left   
        ]

        for dx, dy, cost in valid_moves:
            nx, ny = current_x + dx, current_y + dy
            if ((0 <= nx < self.map_width) and (0 <= ny < self.map_height)):
                if (self.map_data[ny, nx] is not None and self.map_data[ny, nx] < LETHAL_COST):
                    new_cost = cost + (self.map_data[ny, nx] / 255)
                    neighbours.append(((nx, ny), new_cost))

        return neighbours

        
    def construct_path(self, came_from, current):
        path = []
        while current in came_from:
            path.append(current)
            current = came_from[current]
        path.reverse()
        return path

    def A_Algorithm(self, start_x, start_y, goal_x, goal_y):

        open_set = []
        heapq.heappush(open_set, (0, (start_x, start_y)))

        g_score = {(start_x, start_y): 0}
        f_score = {(start_x, start_y): self.h(start_x,start_y, goal_x, goal_y)}
        came_from = {}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == (goal_x, goal_y):
                return self.construct_path(came_from, current)
            
            for neigbour, cost in self.valid_nodes(current):
                temp_g_cost = g_score[current] + cost
                if neigbour not in g_score or temp_g_cost < g_score[neigbour]:
                    came_from[neigbour] = current
                    g_score[neigbour] = temp_g_cost
                    f_score[neigbour] = temp_g_cost + self.h(neigbour[0], neigbour[1], goal_x, goal_y)

                    heapq.heappush(open_set, (f_score[neigbour], neigbour))

        return []
    
    def smooth_path(self, path, samples=40):
        if not path or len(path) < 2:
            return path
        
        smoothed = []
        smoothed.append(path[0])

        for i in range(1, len(path)-1, 2):
            p0 = path[i - 1]
            p1 = path[i]
            p2 = path[i + 1]

            for t in np.linspace(0, 1, samples):
                x = (1-t)**2 * p0[0] + 2*(1-t)*t*p1[0] + t**2 * p2[0] 
                y = (1-t)**2 * p0[1] + 2*(1-t)*t*p1[1] + t**2 * p2[1] 
                smoothed.append((x,y))
        smoothed.append(path[-1])

        return smoothed



    
    def publish_path(self, path):
        if not path or len(path) < 1:
            return
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'

        smoothed_path = self.smooth_path(path)
        path_meters = [self.grid_to_meters(x, y) for x, y in smoothed_path]

        start_pose = PoseStamped()
        start_pose.pose.position.x = float(path_meters[0][0])
        start_pose.pose.position.y = float(path_meters[0][1])
        start_pose.pose.orientation.w = 1.0
        path_msg.poses.append(start_pose)

        for i in range(1, len(path_meters)):
            pose = PoseStamped()
            pose.pose.position.x = path_meters[i][0]
            pose.pose.position.y = path_meters[i][1]

            dx = path_meters[i][0] - path_meters[i-1][0]
            dy = path_meters[i][1] - path_meters[i-1][1]
            theta = np.arctan2(dy, dx)

            cy = np.cos(0.5*theta)
            sy = np.sin(0.5*theta)
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = sy
            pose.pose.orientation.w = cy

            path_msg.poses.append(pose)
        
        self.path_pub.publish(path_msg)
            


def main(args=None):
    rclpy.init(args=args)
    planner = Planner()
    try:
        rclpy.spin(planner)
    except KeyboardInterrupt:
        pass
    finally:
        planner.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



