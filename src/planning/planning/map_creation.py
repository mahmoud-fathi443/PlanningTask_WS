import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np

class Map(Node):
    def __init__(self):
        super().__init__("map_creation")

        self.map_pub = self.create_publisher(OccupancyGrid, "my_map", 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.map_size = 100 
        self.map_resolution = 0.05 
        
        self.grid = OccupancyGrid()

        #Generates the map: empty marked with 0 and random obstacles marked with 100 but at with a chance of 20% to allow for path planning
        self.map = np.random.choice([0, 100], size=(self.map_size, self.map_size), p=[0.8, 0.2]).astype(np.int8)
        


    def timer_callback(self):
        self.grid.header.stamp = self.get_clock().now().to_msg()
        self.grid.header.frame_id = "map"
        self.grid.info.resolution = self.map_resolution
        self.grid.info.width = self.map_size
        self.grid.info.height = self.map_size

        self.grid.info.origin.position.x = 0.0
        self.grid.info.origin.position.y = 0.0
        self.grid.info.origin.position.z = 0.0

        self.grid.data = self.map.flatten().tolist()

        self.map_pub.publish(self.grid)

        
def main(args=None):
    rclpy.init(args=args)
    map = Map()
    try:
        rclpy.spin(map)
    except KeyboardInterrupt:
        pass
    finally:
        map.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()



