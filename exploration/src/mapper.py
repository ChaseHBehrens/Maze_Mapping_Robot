#!/usr/bin/env python3

from __future__ import annotations
from typing import List, Tuple, Iterable, Set

import math
import rclpy
import yaml
import numpy as np
import os
from PIL import Image
from pathlib import Path
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, GridCells, Odometry
from std_msgs.msg import Header
import tf2_geometry_msgs
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

class Mapper(Node):
    def __init__(self):
        
        # Initialize node, name it "mapper"
        super().__init__('mapper')
        
        # Create callback group
        self.cb_group = ReentrantCallbackGroup()
        self.cb_group_path = MutuallyExclusiveCallbackGroup()
        
        # Create Quality of Service (QoS) policies
        self.QoS_Policy_map = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10)
        self.QoS_Policy_rviz = QoSProfile(durability=DurabilityPolicy.VOLATILE, depth=10)

        # Subscribe to '/map', /move_base_simple/goal', '/clicked_point', and '/odom'
        self.create_subscription(
            OccupancyGrid, 
            '/map', 
            self.handle_map, 
            self.QoS_Policy_map, 
            callback_group=self.cb_group
        )
        self.create_subscription(
            Odometry, 
            '/odom', 
            self.update_odometry,
            10, 
            callback_group = self.cb_group
        )
        
        # Publishers
        self.highlight = self.create_publisher(
            GridCells, 
            '/mapper/highlight', 
            self.QoS_Policy_map
        )
        self.frontiers = self.create_publisher(
            GridCells,
            '/mapper_frontiers',
            self.QoS_Policy_map
        )

        # Create a transform listener + TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize attributes
        self.origin = PoseStamped()

        self.home_pose = None

        self.mapdata = None
        
        self.map_done = False
        self.create_timer(1, self.run_find_frontiers, callback_group=self.cb_group_path)
        
        self.cspace_padding = 6
        self.aspace_padding = 7
        

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: tuple[int, int]) -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        
        # Is the point within the bounds of the map?
        if p[0] >= mapdata.info.width or p[1] >= mapdata.info.height or min(p[0], p[1]) < 0:
            return -1
        
        # Offset in y with map width, then add x
        return int(p[1]*mapdata.info.width + p[0])


    @staticmethod
    def xy_enumerate(mapdata: OccupancyGrid) -> Iterable[tuple[int, int, int]]:
        for i, value in enumerate(mapdata.data):
            x = i % mapdata.info.width
            y = i // mapdata.info.width
            yield x, y, value


    @staticmethod
    def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        
        # Calculate the change in x and y
        x_diff = p2[0] - p1[0]
        y_diff = p2[1] - p1[1]
        
        # Calculate using the distance formula
        return math.sqrt(math.pow(x_diff, 2) + math.pow(y_diff, 2))


    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        
        # Scale the coordinate pair to the world's resolution
        scaled = [int(p[0])*mapdata.info.resolution, int(p[1])*mapdata.info.resolution]
        
        # Translate the coordinate pair to its appropriate cell
        x = scaled[0] + mapdata.info.origin.position.x
        y = scaled[1] + mapdata.info.origin.position.y
        
        # Assign world point values, centering x and y within its cell
        return Point(x=(x + mapdata.info.resolution/2),
                     y=(y + mapdata.info.resolution/2),
                     z=0.0)


    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> tuple[int, int]:
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        
        # Translate x and y 
        trans_x = wp.x - mapdata.info.origin.position.x
        trans_y = wp.y - mapdata.info.origin.position.y
        
        # Scale and floor x and y to give them integer values
        return (int(trans_x // mapdata.info.resolution), int(trans_y // mapdata.info.resolution))
    

    @staticmethod
    def is_open_cell(mapdata: OccupancyGrid, cell: Tuple[int, int]) -> bool: 
        '''
        Determine if a given cell is open for the robot to occupy (index is less than 100)
        :param mapdata [OccupancyGrid] The map information.
        :param cell    [(int, int)]    The given cell to check.
        :return        bool            Return True if the cell is within the map and has an index less than 100
        '''
        
        # Determine if a given cell is accessible in a given map
        index = Mapper.grid_to_index(mapdata, cell)
        if index < 0 or index >= len(mapdata.data): return False
        
        # Determine if the index is less than 100
        return not(mapdata.data[index] == 100)
    
    
    
    def update_odometry(self, msg: Odometry):
        '''
        A callback that updates the current pose of the robot
        :param msg [Odometry] the current odometry information
        ''' 
        
        if self.mapdata == None: return
        
        # transform the odom data to map frame
        pose_in_odom = PoseStamped()
        pose_in_odom.header.stamp = Time(seconds=0).to_msg()
        pose_in_odom.header.frame_id = 'odom'  # THIS IS THE KEY LINE - explicitly set to 'odom'
        pose_in_odom.pose.position.x = float(msg.pose.pose.position.x)
        pose_in_odom.pose.position.y = float(msg.pose.pose.position.y)
        pose_in_odom.pose.position.z = float(0.0) 

        self.origin = self.tf_buffer.transform(pose_in_odom, 'map', timeout=rclpy.duration.Duration(seconds=1))
        if self.home_pose == None:
            self.home_pose = self.origin
        

    def calc_cspace(self, mapdata: OccupancyGrid, padding: int) -> OccupancyGrid:
        """
        Calculates the C-Space, i.e., makes the obstacles in the map thicker.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :param padding [int]           The number of cells around the obstacles.
        :return        [OccupancyGrid] The C-Space.
        """
        
        # Create an OccupancyGrid to fill with our cspace
        cspace = OccupancyGrid()
        cspace.header = Header()
        cspace.header.stamp = mapdata.header.stamp
        cspace.header.frame_id = mapdata.header.frame_id
        
        # Assign cspace info and data
        cspace.info = mapdata.info
        cspace.data = list(mapdata.data)
        
        for x, y, value in self.xy_enumerate(mapdata):
            
            # Determine if a cell is completely walkable, undiscovered, or doesn't neighbor other obstacles
            walkable = value == 0
            undiscovered = value == -1
            no_neighbors = len(Mapper.neighbors_of_4(mapdata, (x, y))) == 4
            
            # If any of these are true, ignore the cell
            if walkable or undiscovered or no_neighbors: continue
            
            # Determine the upper and lower bounds in the x and y directions
            lower_bound_x = max(0, x - padding)
            upper_bound_x = min(mapdata.info.width, x + padding + 1)
            lower_bound_y = max(0, y - padding)
            upper_bound_y = min(mapdata.info.height, y + padding + 1)
            
            # Check all cells within the padding distance in x and y
            for mask_x in range(lower_bound_x, upper_bound_x):
                for mask_y in range(lower_bound_y, upper_bound_y):
                    
                    # For each cell within the padding in the x and y directions, determine
                    # if the cell is within the padding radius of its parent cell
                    if math.sqrt((mask_x-x)**2 + (mask_y-y)**2) < (padding - 0.2):
                        # Make the cell an obstacle
                        cspace.data[Mapper.grid_to_index(mapdata, (mask_x, mask_y))] = 100
        
        return cspace


    def handle_map(self, mapdata: OccupancyGrid):
        '''
        Upon receiving a map, determine the three nearest frontiers
        to the robot and send their cells to the path planner
        :param mapdata [OccupancyGrid] The map information.
        '''
        self.mapdata = mapdata 
        if self.home_pose:
            self.home_pose = self.tf_buffer.transform(self.home_pose, 'map', timeout=rclpy.duration.Duration(seconds=1))


    def run_find_frontiers(self):
        if self.mapdata == None: return
        # if self.map_done: return
        
        # Create a list of all existing frontiers
        best_frontiers = [self.grid_to_world(self.mapdata, point) for point in self.find_frontiers(self.mapdata)]
        
        self.get_logger().info(f"number of frontiers: {len(best_frontiers)}")
        # if len(best_frontiers) == 0 and self.home_pose:
            
        #     self.get_logger().info("saving map")
        #     # self.save_map(self.mapdata)
        #     self.map_done = True
        #     best_frontiers = [self.home_pose.pose.position]
        
        # Create the GridCells message
        msg_fronts = GridCells()
        msg_fronts.header = Header()
        msg_fronts.header.stamp = self.get_clock().now().to_msg()
        msg_fronts.header.frame_id = self.mapdata.header.frame_id
        
        # Assign list of cells and parameters
        msg_fronts.cell_height = float(self.mapdata.info.resolution)
        msg_fronts.cell_width = float(self.mapdata.info.resolution)
        msg_fronts.cells = best_frontiers
        
        self.get_logger().info("Publishing frontiers")
        # Publish to /mapper_frontiers for path5planner to receive
        self.frontiers.publish(msg_fronts)


    @staticmethod
    def neighbors_of_4(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 4-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int, int)]]   A list of walkable 4-neighbors.
        """
        
        # Assign a point to each cardinal direction
        north = (p[0], p[1] + 1)
        east  = (p[0] + 1, p[1])
        south = (p[0], p[1] - 1)
        west  = (p[0] - 1, p[1])
        
        # Create list of points
        l = [north, east, south, west]
        
        # Only return members of the list if they are walkable cells
        return list(filter(lambda x: Mapper.is_cell_walkable(mapdata, x), l))


    @staticmethod
    def neighbors_of_8(mapdata: OccupancyGrid, p: tuple[int, int]) -> list[tuple[int, int]]:
        """
        Returns the walkable 8-neighbors cells of (x,y) in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [[(int,int)]]   A list of walkable 8-neighbors.
        """
        
        # Assign a point to each cardinal and intercardinal direction
        north     = (p[0], p[1] + 1)
        northeast = (p[0] + 1, p[1] + 1)
        east      = (p[0] + 1, p[1])
        southeast = (p[0] + 1, p[1] - 1)
        south     = (p[0], p[1] - 1)
        southwest = (p[0] - 1, p[1] - 1)
        west      = (p[0] - 1, p[1])
        northwest = (p[0] - 1, p[1] + 1)
        
        # Create list of points
        l = [north, northeast, east, southeast, south, southwest, west, northwest]
        
        # Only return members of the list if they are walkable cells
        return list(filter(lambda x: mapdata.data[Mapper.grid_to_index(mapdata, x)] == 0, l))


    def find_frontiers(self, mapdata: OccupancyGrid) -> List[Tuple[int, int]]:
        '''
        Finds the center of all frontiers in a map. p saved to small_field.yaml and small_field.pg
        :param mapdata [OccupancyGrid] The map information.
        :return        [(x, y)]        The coordinates of all frontiers.
        '''
        
        # Create a list of all accessible cells that border unexplored cells
        borders = self.find_borders(mapdata)

        # Prune inaccessible frontiers by calculating the cspace
        cspace = self.calc_cspace(mapdata, self.cspace_padding)
        aspace = self.calc_cspace(mapdata, self.aspace_padding)
        open_borders = self.prune_unwalkables(cspace, aspace, borders)
        
        # Find the centers of each cluster of frontiers, making path planning more efficient
        clusters = Mapper.find_clusters(set(open_borders))
        centroids = Mapper.find_centroids(clusters)
        
        # Reprune in case centroids are unwalkable
        #valid_centroids = self.prune_unwalkables(cspace, aspace, centroids)
        
        # Create a GridCells message containing our frontiers
        msg_map = GridCells()
        msg_map.header = Header()
        msg_map.header.stamp = self.get_clock().now().to_msg()
        msg_map.header.frame_id = 'map'
        
        # Assign list of cells and parameters
        msg_map.cell_width = float(mapdata.info.resolution)
        msg_map.cell_height = float(mapdata.info.resolution)
        msg_map.cells = [self.grid_to_world(mapdata, c) for c in open_borders]
        
        # Publish to /mapper/highlight to visualize frontiers
        self.highlight.publish(msg_map)

        return centroids

    
    @staticmethod
    def is_cell_walkable(mapdata:OccupancyGrid, p: tuple[int, int]) -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        
        # Determine the index of point p in mapdata
        index = Mapper.grid_to_index(mapdata, p)
        
        # Is the point within the boundaries of the grid?
        in_boundary = p[0] < mapdata.info.width and p[1] < mapdata.info.height and min(p[0], p[1]) >= 0
        
        # Is the point known?
        unknown = mapdata.data[index] == -1
        
        # Is the point unoccupied?
        unoccupied = not(mapdata.data[index] == 100)
        
        return in_boundary and (unoccupied or unknown)


    def find_borders(self, mapdata: OccupancyGrid) -> Set[Tuple[int, int]]:
        '''
        find_frontiers helper function
        Finds the cells bordering explored area.
        '''
        
        # Save the explored cells that are walkable
        open_cells = set()
        for x, y, value in self.xy_enumerate(mapdata):
            if value == 0 and self.euclidean_distance(Mapper.world_to_grid(mapdata, self.origin.pose.position), (x, y)) > 12:
                open_cells.add((x, y))

        # For each saved cell, determine if it has at least one neighbor that is unexplored
        def unexplored_neighbor(p: tuple[int, int]) -> bool:
            north = (p[0], p[1] + 1)
            east  = (p[0] + 1, p[1])
            south = (p[0], p[1] - 1)
            west  = (p[0] - 1, p[1])
            neighbors_values = [
                mapdata.data[Mapper.grid_to_index(mapdata, cell)]
                for cell in [north, east, south, west]
            ]
            return -1 in neighbors_values
        
        # Return a list of saved cells that border an unexplored cell
        return [cell for cell in open_cells if unexplored_neighbor(cell)]


    @staticmethod
    def find_clusters(borders: Set[Tuple[int, int]]) -> List[List[Tuple[int, int]]]:
        '''
        find_frontiers helper function
        Groups bordering cells into groups of cells that touch each other
        '''
        clusters = []
        que = []
        while True:
            if len(que) == 0:
                # return when all cells are clustered
                if len(borders) == 0:
                    return clusters
                # create a new cluster
                que.append(borders.pop())
                clusters.append([])
            
            # put the current cell being processed into the current cluster
            cell = que.pop()
            clusters[-1].append(cell)
            
            # move neighbors from borders into que
            north     = (cell[0], cell[1] + 1)
            northeast = (cell[0] + 1, cell[1] + 1)
            east      = (cell[0] + 1, cell[1])
            southeast = (cell[0] + 1, cell[1] - 1)
            south     = (cell[0], cell[1] - 1)
            southwest = (cell[0] - 1, cell[1] - 1)
            west      = (cell[0] - 1, cell[1])
            northwest = (cell[0] - 1, cell[1] + 1)
            neighbors = {north, northeast, east, southeast, south, southwest, west, northwest}
            que.extend(neighbors & borders)
            borders -= neighbors & borders


    @staticmethod
    def find_centroids(clusters: list[list[Tuple[int, int]]]) -> List[Tuple[int, int]]:
        '''
        Calculates all the centroids of list of clusters.
        '''
        centroids = []
        for cluster in clusters:
            x_centroid = 0
            y_centroid = 0
            for (x, y) in cluster:
                x_centroid += x
                y_centroid += y
            centroids.append((x_centroid / len(cluster), y_centroid / len(cluster)))

        return centroids


    def prune_unwalkables(self, cspace: OccupancyGrid, aspace: OccupancyGrid, cells: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        '''
        Prunes all cells that are solid obstacles in a list of cells.
        :param mapdata [OccupancyGrid] The map information.
        :param cells   [(int, int)]    List of cells to check.
        :return        [(int, int)]    List of non-obstacle cells.
        '''
        
        # Create an empty list of walkable cells
        walkable_cells = []
        
        # For each cell, add it to our list of walkable cells if it is not a solid obstacle
        for cell in cells:
            far_away = self.euclidean_distance(Mapper.world_to_grid(cspace, self.origin.pose.position), cell) > 8
            free_cell_cspace = self.is_open_cell(cspace, cell)
            free_cell_aspace = self.is_open_cell(aspace, cell)
            if (far_away and free_cell_cspace) or free_cell_aspace:
                walkable_cells.append(cell)

        return walkable_cells


    

    def save_map(self, map_msg): # Assume map_msg is the OccupancyGrid object
        TARGET_DIR = Path("/home/ros2_ws/src/RBE-3002-B25-Team8/exploration/mapdata")
    # 1. Define the base name for the files
        FILE_BASE_NAME = "small_field"
    
    # 2. Construct the full absolute paths for saving
        pgm_path = TARGET_DIR / f'{FILE_BASE_NAME}.pgm'
        yaml_path = TARGET_DIR / f'{FILE_BASE_NAME}.yaml'
    
    # --- Ensure the directory exists ---
    # The parents=True ensures intermediate directories are created.
    # exist_ok=True prevents an error if the directory already exists.
        TARGET_DIR.mkdir(parents=True, exist_ok=True)
    
    # --- Convert occupancy grid to image (Original Code) ---
        width = map_msg.info.width
        height = map_msg.info.height
    
    # Reshape data
        data = np.array(map_msg.data).reshape((height, width))
        data = np.flipud(data)
    # Convert to image format (0-255)
        img_data = np.zeros_like(data, dtype=np.uint8)
        img_data[data == -1] = 205  # Unknown = gray
        img_data[data == 0] = 254   # Free = white
        img_data[data == 100] = 0   # Occupied = black
    
    # Save image
        img = Image.fromarray(img_data)
        img.save(pgm_path) # Use the full path object here
    
    # --- Save YAML metadata (Corrected) ---
        yaml_data = {
        # CRITICAL: This must remain only the filename, NOT the full path.
            'image': f'{FILE_BASE_NAME}.pgm', 
            'resolution': map_msg.info.resolution,
            'origin': [
                map_msg.info.origin.position.x,
                map_msg.info.origin.position.y,
                0.0
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
    
        with open(yaml_path, 'w') as f: # Use the full path object here
            yaml.dump(yaml_data, f)
        
        self.get_logger().info(f'Map saved to {yaml_path} and {pgm_path}')


def main(args=None):
    rclpy.init(args=args)
    
    node = Mapper()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: # when 'ctrl + C' is pressed
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
