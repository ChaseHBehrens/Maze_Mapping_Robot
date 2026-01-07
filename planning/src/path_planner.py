#!/usr/bin/env python3
from __future__ import annotations
from typing import List, Tuple, Iterable

import math
import numpy as np
import cv2
import rclpy
import copy
import tf2_geometry_msgs
from rclpy.time import Time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, PointStamped, PoseStamped, Pose, Quaternion
from nav_msgs.msg import OccupancyGrid, Path, GridCells, Odometry
from std_msgs.msg import Header, String
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from priority_queue import PriorityQueue
from scipy.spatial.transform import Rotation 

class PathPlanner(Node):
    def __init__(self):
        
        # Initialize node, name it "path_planner"
        super().__init__('path_planner')
        
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
        self.create_subscription(
            GridCells,
            '/mapper_frontiers',
            self.handle_frontiers,
            self.QoS_Policy_map,
            callback_group = self.cb_group
        )
        
        # Publish to '/path_planner/c_obstacle', '/path_planner/highlight', and '/nav_path'
        self.gridCellPublisher = self.create_publisher(
            GridCells, 
            '/path_planner/c_obstacle', 
            self.QoS_Policy_map
        )
        self.occupancyPublisher = self.create_publisher(
            OccupancyGrid, 
            '/path_planner/gaussian_blur', 
            self.QoS_Policy_map
        )
        self.clickHighlight = self.create_publisher(
            GridCells, 
            '/path_planner/highlight', 
            self.QoS_Policy_map
        )
        self.navPathPublisher = self.create_publisher(
            Path, 
            '/nav_path', 
            self.QoS_Policy_map
        )
        self.faultPublisher = self.create_publisher(
            String,
            '/fault',
            self.QoS_Policy_map
        )
        self.robotPositionPublisher = self.create_publisher(
            Odometry,
            '/path_planner/robot_position',
            self.QoS_Policy_map
        )
        

        # Create a transform listener + TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Initialize attributes
        self.origin = PoseStamped()
        self.mapdata = None
        self.cspace_padding = 6
        self.aspace_padding = 7
        self.goal_pose = None
        self.recompute = True
        self.path = None
        self.last_pose_time_stamp = self.get_clock().now().to_msg()

        self.create_timer(1.5, self.plan_path, callback_group=self.cb_group_path)



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
        return p[1]*mapdata.info.width + p[0]


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
       

    def handle_frontiers(self, frontiers: GridCells):
        '''
        Finds the point with the shortest A* path from the robot
        :param frontiers [GridCells] The list of frontiers.
        '''
        # Recalculate the cspace to ensure correct and efficient paths
        cspace = self.calc_cspace(self.mapdata, self.cspace_padding)

        # if the current goal is in the new list of frontiers keep it
        if self.goal_pose != None:
            grid_goal = PathPlanner.world_to_grid(self.mapdata, self.goal_pose.position)
            curr_goal_frontier = PathPlanner.neighbors_of_8(self.mapdata, grid_goal)
            curr_goal_frontier.append(grid_goal)
            grid_frontiers = [PathPlanner.world_to_grid(self.mapdata, cell) for cell in frontiers.cells]
            matching = set(curr_goal_frontier) & set(grid_frontiers)

            if len(matching) > 0:
                new_goal_pose = Pose()
                new_goal_pose.position = PathPlanner.grid_to_world(cspace, matching.pop())
                self.goal_pose = new_goal_pose
                self.recompute = False
                return
            else:
                self.recompute = True
        
        # Initialize a shortest distance and best cell
        best_euclid = float(999)
        best_cell = None
        
        if self.goal_pose != None:
            goal_grid = PathPlanner.world_to_grid(cspace, self.goal_pose.position)
        else:
            goal_grid = PathPlanner.world_to_grid(cspace, self.origin.pose.position)

        for point in frontiers.cells:
            
            # Convert the point to a cell in the grid
            cell = PathPlanner.world_to_grid(cspace, point)
            
            if PathPlanner.is_cell_walkable(cspace, cell): 
                # Determine the A* path distance from the robot to the current cell
                # astar_distance = self.astar_distance(cspace, goal_grid, cell)
                euclidean_distance = self.euclidean_distance(goal_grid, cell)
                
                #self.get_logger().info(f"Euclidean distance for point ({cell[0]}, {cell[1]}): {best_euclid}")
                
                # If the current path's distance is shorter than the current shortest
                if euclidean_distance < best_euclid: 
                    # Assign a new shortest distance and best cell
                    best_euclid = euclidean_distance
                    best_cell = cell
                    
        
        if best_cell == None: return
        
        # Create a GridCells message containing the best cell
        msg_cell = GridCells()
        msg_cell.header = Header()
        msg_cell.header.stamp = self.get_clock().now().to_msg()
        msg_cell.header.frame_id = self.mapdata.header.frame_id
        
        # Assign list of cells and parameters
        msg_cell.cell_width  = self.mapdata.info.resolution
        msg_cell.cell_height = self.mapdata.info.resolution
        msg_cell.cells = [self.grid_to_world(self.mapdata, best_cell)]
        
        # Publish the best cell to /path_planner/highlight
        self.clickHighlight.publish(msg_cell)
           
        # Create a new point with our nearest cell
        new_point = self.grid_to_world(self.mapdata, best_cell)
        
        # Set the goal pose to our shortest path
        new_goal_pose = Pose()
        new_goal_pose.position = new_point
        self.goal_pose = new_goal_pose
         

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: tuple[int, int]) -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        
        # Scale the coordinate pair to the world's resolution
        scaled = [p[0]*mapdata.info.resolution, p[1]*mapdata.info.resolution]
        
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
    def mapdata_to_image(mapdata: OccupancyGrid) -> np.NDArray:
        width = mapdata.info.width
        height = mapdata.info.height
        return np.array(mapdata.data, dtype=np.uint8).reshape((height, width))
        
    
    def path_to_poses(self, mapdata: OccupancyGrid, path: list[tuple[int, int]], finalTheta: float) -> list[PoseStamped]:
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        
        # Making output array
        poses = []
        
        # Set the heading of each pose in the direction of the next point in the path
        for i in range(len(path)):
            
            # Determine the current coordinates in the world frame
            wp = PathPlanner.grid_to_world(mapdata, path[i])
            
            # The last point in the list will be assigned a theta of zero
            if i == len(path) - 1:
                theta = finalTheta
            else:
                
                # Determine the next point's coordinates in the world frame
                next_wp = PathPlanner.grid_to_world(mapdata, path[i+1])
                
                # Calculate the angle between the current and next point
                theta = math.atan2(next_wp.y - wp.y, next_wp.x - wp.x)
            
            # Assign the angle to a quaternion
            quat = Quaternion(
                x=0.0,
                y=0.0,
                z=float(math.sin(theta/2)),
                w=float(math.cos(theta/2))
            )

            # Create a stamped pose
            pose = PoseStamped()
            
            # Create a header for the pose
            pose.header = Header()
            pose.header.frame_id = self.mapdata.header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # Assign position and orientation values for the pose
            pose.pose.position = wp
            pose.pose.orientation = quat
            
            # Adding pose to the list of poses 
            poses.append(pose)
            
        return poses

    
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
        return list(filter(lambda x: PathPlanner.is_cell_walkable(mapdata, x), l))
    

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
        return list(filter(lambda x: PathPlanner.is_cell_walkable(mapdata, x), l))


    @staticmethod
    def nearest_walkable_cell(mapdata: OccupancyGrid, p: tuple[int, int], padding: int) -> tuple[int, int]:
        
        smallest_distance = float(999)
        best_cell = None
        
        x_range = range(p[0] - padding, p[0] + padding + 1)
        y_range = range(p[1] - padding, p[1] + padding + 1)
        
        for x in x_range:
            for y in y_range:
                
                walkable = PathPlanner.is_cell_walkable(mapdata, (x, y))
                
                cell_distance = PathPlanner.euclidean_distance(p, (x, y))
                
                if cell_distance < smallest_distance and walkable:
                    smallest_distance = cell_distance
                    best_cell = (x, y)
        
        return best_cell


    @staticmethod
    def fov(mapdata: OccupancyGrid, origin: Tuple[int, int]) -> List[Tuple[int, int]]:
        """
        Finds a list of all cells in line of sight of the orgin cell.Goal is an unwalkable cell (33, 19)
        :param mapdata [OccupancyGrid] The map data
        :param origin [(x, y)] The origin cell to check field of view from
        :return [[(x, y)]] All cells in line of sight to the origin cell
        """
        fov = [origin]

        max_depth = 5

        NORTH = 0
        EAST = 1
        SOUTH = 2
        WEST = 3
        def scan(
            direction: int, 
            origin: Tuple[int, int], 
            depth: int =1, 
            start_slope: float =-3, 
            end_slope: float =3, 
            start_edge: bool =True,
            end_edge: bool =True
        ): 
            if depth > max_depth: return
            if direction == NORTH:
                row = [
                    (origin[0] + c, origin[1] + depth) 
                    for c in range(
                        math.ceil(((depth + 0.5) * start_slope) - 0.5) 
                            if abs(start_slope) < 0 else 
                            math.ceil(((depth - 0.5) * start_slope) + 0.5), 
                        1 + (math.floor(((depth + 0.5) * end_slope) + 0.5) 
                            if abs(end_slope) < 0 else 
                            math.floor(((depth - 0.5) * end_slope) - 0.5))
                    )
                ]
            if direction == SOUTH:
                row = [
                    (origin[0] + c, origin[1] - depth)
                    for c in range(
                        math.ceil(((depth + 0.5) * start_slope) - 0.5) 
                            if abs(start_slope) < 0 else 
                            math.ceil(((depth - 0.5) * start_slope) + 0.5),
                        1 + (math.floor(((depth + 0.5) * end_slope) + 0.5) 
                            if abs(end_slope) < 0 else 
                            math.floor(((depth - 0.5) * end_slope) - 0.5))
                    )
                ]
            if direction == EAST:
                row = [
                    (origin[0] + depth, origin[1] + c)
                    for c in range(
                        math.ceil(((depth + 0.5) * start_slope) - 0.5) 
                            if abs(start_slope) < 0 else 
                            math.ceil(((depth - 0.5) * start_slope) + 0.5),
                        1 + (math.floor(((depth + 0.5) * end_slope) + 0.5) 
                            if abs(end_slope) < 0 else
                            math.floor(((depth - 0.5) * end_slope) - 0.5))
                    )
                ]
            if direction == WEST:
                row = [
                    (origin[0] - depth, origin[1] + c)
                    for c in range(
                        math.ceil(((depth + 0.5) * start_slope) - 0.5) 
                            if abs(start_slope) < 0 else 
                            math.ceil(((depth - 0.5) * start_slope) + 0.5), 
                        1 + (math.floor(((depth + 0.5) * end_slope) + 0.5) 
                            if abs(end_slope) < 0 else 
                            math.floor(((depth - 0.5) * end_slope) - 0.5))
                    )
                ]
            if not row: return
            if start_edge:
                if direction == NORTH: start_slope = ((row[0][0] - origin[0]) - 1.5) / (depth + 0.5)
                if direction == SOUTH: start_slope = ((row[0][0] - origin[0]) - 1.5) / (depth + 0.5)
                if direction == EAST: start_slope = ((row[0][1] - origin[1]) - 1.5) / (depth + 0.5)
                if direction == WEST: start_slope = ((row[0][1] - origin[1]) - 1.5) / (depth + 0.5)
            if end_edge:
                if direction == NORTH: end_slope = ((row[-1][0] - origin[0]) + 1.5) / (depth + 0.5)
                if direction == SOUTH: end_slope = ((row[-1][0] - origin[0]) + 1.5) / (depth + 0.5)
                if direction == EAST: end_slope = ((row[-1][1] - origin[1]) + 1.5) / (depth + 0.5)
                if direction == WEST: end_slope = ((row[-1][1] - origin[1]) + 1.5) / (depth + 0.5)
            prev_walkable = None
            final_scan_needed = True
            for cell in row:
                curr_walkable = PathPlanner.is_cell_walkable(mapdata, cell)
                if direction == NORTH: 
                    if ((cell[0] - origin[0]) - 0.5) / depth < 0:
                        slope2 = ((cell[0] - origin[0]) - 0.5) / (depth + 0.5)
                        slope1 = ((cell[0] - origin[0]) - 0.5) / (depth - 0.5)
                    else: 
                        slope1 = ((cell[0] - origin[0]) - 0.5) / (depth + 0.5)
                        slope2 = ((cell[0] - origin[0]) - 0.5) / (depth - 0.5)
                if direction == SOUTH: 
                    if ((cell[0] - origin[0]) - 0.5) / depth < 0:
                        slope2 = ((cell[0] - origin[0]) - 0.5) / (depth + 0.5)
                        slope1 = ((cell[0] - origin[0]) - 0.5) / (depth - 0.5)
                    else: 
                        slope1 = ((cell[0] - origin[0]) - 0.5) / (depth + 0.5)
                        slope2 = ((cell[0] - origin[0]) - 0.5) / (depth - 0.5)
                if direction == EAST:
                    if ((cell[1] - origin[1]) - 0.5) / depth < 0:
                        slope2 = ((cell[1] - origin[1]) - 0.5) / (depth + 0.5)
                        slope1 = ((cell[1] - origin[1]) - 0.5) / (depth - 0.5)
                    else: 
                        slope1 = ((cell[1] - origin[1]) - 0.5) / (depth + 0.5)
                        slope2 = ((cell[1] - origin[1]) - 0.5) / (depth - 0.5)
                if direction == WEST:
                    if ((cell[1] - origin[1]) - 0.5) / depth < 0:
                        slope2 = ((cell[1] - origin[1]) - 0.5) / (depth + 0.5)
                        slope1 = ((cell[1] - origin[1]) - 0.5) / (depth - 0.5)
                    else: 
                        slope1 = ((cell[1] - origin[1]) - 0.5) / (depth + 0.5)
                        slope2 = ((cell[1] - origin[1]) - 0.5) / (depth - 0.5)
                if (curr_walkable == True) and (prev_walkable == False):
                    start_slope = slope2
                    start_edge = False
                    final_scan_needed = True
                if (curr_walkable == False) and (prev_walkable == True):
                    end_edge = False
                    scan(direction, origin, depth + 1, start_slope, slope1, start_edge, end_edge)
                    final_scan_needed = False
                if curr_walkable:
                    prev_walkable = True
                    fov.append(cell)
                else:
                    prev_walkable = False
            if final_scan_needed and prev_walkable:
                scan(direction, origin, depth + 1, start_slope, end_slope, start_edge, end_edge)

        scan(NORTH, origin)
        scan(SOUTH, origin)
        scan(EAST, origin)
        scan(WEST, origin)
        
        return fov


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
        index = PathPlanner.grid_to_index(mapdata, p)
        
        # Is the point within the boundaries of the grid?
        in_boundary = p[0] < mapdata.info.width and p[1] < mapdata.info.height and min(p[0], p[1]) >= 0
        
        # Is the point known?
        unknown = mapdata.data[index] == -1
        
        # Is the point unoccupied?
        unoccupied = not(mapdata.data[index] == 100)
        
        return in_boundary and (unoccupied or unknown)


    def handle_map(self, mapdata:OccupancyGrid):
        """
        Store the incoming map data in a global variable. 
        This function should be called and the map data updated whenever it is available.
        """
        
        # Setting info into the mapdata global variable
        self.mapdata = mapdata 

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
            no_neighbors = len(PathPlanner.neighbors_of_4(mapdata, (x, y))) == 4
            
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
                    if math.sqrt((mask_x-x)**2 + (mask_y-y)**2) < (padding - 0.1):
                        # Make the cell an obstacle
                        cspace.data[PathPlanner.grid_to_index(mapdata, (mask_x, mask_y))] = 100
        
        if padding == self.cspace_padding:
            # Create a GridCells message containing the new cspace
            msg_gridcells = GridCells()
            msg_gridcells.header = Header()
            msg_gridcells.header.stamp = self.get_clock().now().to_msg()
            msg_gridcells.header.frame_id = mapdata.header.frame_id
            
            # Assign list of cells and parameters
            msg_gridcells.cell_width  = mapdata.info.resolution
            msg_gridcells.cell_height = mapdata.info.resolution
            msg_gridcells.cells = [
                self.grid_to_world(mapdata, (x, y))
                for x, y, value in self.xy_enumerate(cspace)
                if value == 100
            ]
            
            # Publish this cspace to /path_planner/c_obstacle
            self.gridCellPublisher.publish(msg_gridcells)
        
        return cspace


    def calc_bspace(self, mapdata: OccupancyGrid) -> OccupancyGrid:
        """
        Calculates the B-Space, i.e., blurs the edge of obstacles.
        Publishes the list of cells that were added to the original map.
        :param mapdata [OccupancyGrid] The map data.
        :return        [OccupancyGrid] The B-Space.
        """
        
        # Create an image of the mapdata
        image = PathPlanner.mapdata_to_image(mapdata)
        
        # Create a blurred version of this image using Gaussian blurring
        gaussian = np.clip(cv2.GaussianBlur(image, (7, 7), 0.5).flatten(), 0, 100).astype(np.int8)
        
        # Create a GridCells message containing the new bspace
        bspace = OccupancyGrid()
        bspace.header = Header()
        bspace.header.stamp = self.get_clock().now().to_msg()
        bspace.header.frame_id = mapdata.header.frame_id
        
        # Assign list of cells and parameters
        bspace.info = mapdata.info
        bspace.data = gaussian.tolist()
                
        # Publish this bspace to /path_planner/gaussian_blur
        self.occupancyPublisher.publish(bspace)

        return bspace


    def a_star(self, mapdata: OccupancyGrid, bspace: OccupancyGrid, start: tuple[int, int], goal: tuple[int, int], recursive_pad: int =-1) -> list[tuple[int, int]]:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not unknown, not occupied by an obstacle)
        :param mapdata [OccupancyGrid]         The map data.
        :param start   [tuple[int, int]]       The start coordinate in the grid.
        :param goal    [tuple[int, int]]       The goal coordinate in the grid.
        :return        [list[tuple[int, int]]] The optimal path to be followed.
        """
        if self.aspace_padding + recursive_pad < 0: return []
        #max_path_length = 22
        
        #self.get_logger().info("Executing A* from (%d,%d) to (%d,%d)" % (start[0], start[1], goal[0], goal[1]))

        # Don't calculate anything if we're already at the destination
        if goal == start: return []
        
        # Determine if the start is unwalkable to avoid taking too long to find no solution
        if not PathPlanner.is_cell_walkable(mapdata, start):
            
            self.get_logger().info("Robot is in an unwalkable cell")
            
            # Set the start to the nearest walkable cell to the start with padding 2
            start = self.nearest_walkable_cell(mapdata, start, 2)
            
            # If there is no walkable cell in the padding range, throw an error
            if start == None:
                
                self.get_logger().info("Robot has no walkable neighboring cells")
                
                # Publish an error message to /fault
                msg = String()
                msg.data = "unwalkable start"
                self.faultPublisher.publish(msg)
                
                # Return no solution
                return []
            
        # Determine if the goal is unwalkable to avoid taking too long to find no solution
        if not PathPlanner.is_cell_walkable(mapdata, goal):
            
            self.get_logger().info(f"Goal is an unwalkable cell {goal}")
            
            goal = self.nearest_walkable_cell(mapdata, goal, 3)
            
            if goal == None:
            
                # Publish an error message to /fault
                msg = String()
                msg.data = "unwalkable goal"
                self.faultPublisher.publish(msg)
                
                # Return no solution
                return []
        
        # Initialize queue
        queue = PriorityQueue()
        
        # Add the goal point to the queue
        cost = self.euclidean_distance(goal, start)
        queue.put([start], cost)

        # Create a dictionary containing the best cost to travel to each cell
        best_cost_found = {}
        #best_distance = cost
        #best_context = []

        while True:
            
            # Search the highest priority path's neighboring cells
            context = queue.get()
            current_cell = context[0]

            distance_from_robot = PathPlanner.euclidean_distance(start, current_cell)
            far_away = self.euclidean_distance(goal, start) > 50
            if distance_from_robot < 4 or far_away:
                neighbors = PathPlanner.fov(bspace, current_cell)
            else: 
                neighbors = PathPlanner.fov(mapdata, current_cell)
            
         #   if self.euclidean_distance(current_cell, goal) < best_distance:
         #       best_context = context
         #       best_distance = self.euclidean_distance(current_cell, goal)
            
         #   if len(context) >= max_path_length:
         #       self.get_logger().info("A* execution successful to a closer point")
         #       return best_context[::-1]

            # Return the path if current cell is the start
            if current_cell == goal:
                
                #self.get_logger().info("A* execution successful")
                return context[::-1]
            
            # Add each cell and their costs to the queue
            for cell in neighbors:
                
                # Calculate the cost using euclidean distance and the contextual path's cost
                cost = 0
                for prev_cell, curr_cell in zip(context[:-1], context[1:]):
                    cost += self.euclidean_distance(prev_cell, curr_cell)
                cost += self.euclidean_distance(cell, context[0])
                cost += self.euclidean_distance(cell, goal)
                
                # Add a Gaussian blur to the cost, prioritizing path points that are further from obstacles
                #cost += 0.1 * bspace.data[PathPlanner.grid_to_index(bspace, cell)]
                
                # Is the cell walkable?
                walkable = PathPlanner.is_cell_walkable(mapdata, cell)

                # Is the cell optimal?
                optimal = True
                if cell in best_cost_found:
                    optimal = cost < best_cost_found[cell]

                if walkable and optimal:
                    # Add the cell to its contextual list
                    subpath = [cell] + context
                    
                    # Add cost to best costs fond for the cell
                    best_cost_found[cell] = cost

                    # Add path to the queue
                    queue.put(subpath, cost)
                
            # If the queue has been exhausted, no solution exists
            if queue.empty():
                
                self.get_logger().info("No solution, staying at origin")
                
                # Publish an error message to /fault
                msg = String()
                msg.data = "no path"
                self.faultPublisher.publish(msg)

                aspace = self.calc_cspace(self.mapdata, self.aspace_padding + recursive_pad)
                
                # atempt to plan unsafe path to recover
                return self.a_star(mapdata, aspace, start, goal, recursive_pad - 1)
    
    
    @staticmethod
    def optimize_path(path: List[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """
        Optimizes the path, removing unnecessary intermediate nodes.
        :param path [[(x,y)]] The path as a list of tuples (grid coordinates)
        :return     [[(x,y)]] The optimized path as a list of tuples (grid coordinates)
        """   
        
        # If the input list is empty return nothing
        if len(path) == 0:
            return []
        
        # Create an empty list to be filled with optimized points
        optimized_path = []
        
        # Create a placeholder for the difference between i-1 and i
        diff_prev = (0, 0)
        
        for i in range(len(path)):
            
            # If this is the last cell in the path, add it to the list
            last = i == len(path) - 1
            if last:
                optimized_path.append(path[i])
            
            else:
            
                # Determine the difference between i and i+1
                diff_next = (path[i+1][0] - path[i][0], path[i+1][1] - path[i][1])
            
                # If the differences are NOT the same, add the cell to the list
                if diff_prev != diff_next:
                    optimized_path.append(path[i])
                
                # Reassign the previous difference to the next difference for the next loop iteration
                diff_prev = diff_next
        
        # Removing first index, always adds unnecessary offset
        if len(optimized_path) > 1:
            optimized_path.pop(0)
        
        return optimized_path


    def update_odometry(self, msg: Odometry):
        '''
        A callback that updates the current pose of the robot
        :param msg [Odometry] the current odometry information
        ''' 
        
        if self.mapdata == None: return
        
        pose_in_odom = PoseStamped()
        pose_in_odom.header.stamp = Time(seconds=0).to_msg()
        pose_in_odom.header.frame_id = 'odom'  # THIS IS THE KEY LINE - explicitly set to 'odom'
        pose_in_odom.pose.position.x = float(msg.pose.pose.position.x)
        pose_in_odom.pose.position.y = float(msg.pose.pose.position.y)
        pose_in_odom.pose.position.z = float(0.0)
        
        self.origin = self.tf_buffer.transform(pose_in_odom, 'map', timeout=rclpy.duration.Duration(seconds=1))
        

    def plan_path(self): 
        """
        Plans a path between the current pose and the goal message locations.
        Internally uses A* to plan the optimal path.
        :param req 
        """
        self.get_logger().info("plannning path")
        
        # If the goal doesn't exist yet, make a goal for the robot to move to
        if self.goal_pose == None:
            self.get_logger().info("self.goal_pose = None")
            new_goal_pose = Pose()
            new_goal_pose.position = copy.copy(self.origin.pose.position)
            new_goal_pose.position.x += 1
            self.goal_pose = new_goal_pose

        # Error handling for if there is no map available
        if self.mapdata == None: return #raise ValueError("Mapdata not initialized.")
        if self.origin == None: return #raise ValueError("Origin of robot not initialized.")
        if self.goal_pose.position == self.origin.pose.position: self.get_logger().info("goal pose is the origin")

        # Calculates and publishes the c-space of the map
        cspace = self.calc_cspace(self.mapdata, self.cspace_padding)
        
        # Calculates and publishes the b-space of the map
        aspace = self.calc_cspace(self.mapdata, self.aspace_padding)
        #bspace = self.calc_bspace(aspace)
        # self.occupancyPublisher.publish(bspace)
        
        start = PathPlanner.world_to_grid(cspace, self.origin.pose.position)
        if self.recompute or len(self.path) <= 7:
            goal = PathPlanner.world_to_grid(cspace, self.goal_pose.position)

            if PathPlanner.euclidean_distance(start, goal) > 16:
                # Publish an error message to /fault
                msg = String()
                msg.data = "far goal"
                self.faultPublisher.publish(msg) 
            # Calculate the path using A* and optimize it
            points = self.a_star(cspace, aspace, start, goal)
        else:
            goal = self.path[7]
            points = self.a_star(cspace, aspace, start, goal) + self.path[8:]

        
        # If there is no path created, don't try to follow one
        if len(points) == 0:
            return
        
        # optimized_points = self.optimize_path(points) 
        self.path = points

        # Determine the final theta
        quat = self.goal_pose.orientation
        (_, _, yaw) = Rotation.from_quat([quat.x, quat.y, quat.z, quat.w]).as_euler('xyz', degrees=False)
        
        # Turn the points into a path by making them poses
        path = self.path_to_poses(cspace, points, yaw)
        
        # Create a path message
        path_msg = Path()
        
        # Create a header for the path
        path_msg.header = Header()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.mapdata.header.frame_id
        
        # Assign the path within the list of poses
        path_msg.poses = path
        
        # Publish the path
        self.navPathPublisher.publish(path_msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = PathPlanner()
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
