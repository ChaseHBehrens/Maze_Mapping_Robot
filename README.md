# Maze Mapping Robot

This robot was built on the TurtleBot3 platform and is operated from an external linux computer using ROS2. It is capable of autonomously navigating and mapping unknown environments using simultaneous localization and mapping, SLAM. It is also capable of localizing in a known environment using adaptive monte carlo localization, AMCL. Algorithm refinement was done using Gazebo simulation software. Data visualization and debugging was done using Rviz software. 

The Robot navigates using a multistep process. First a Cspace removes all points inaccessible to the robot from the map. Next unknown points that border known areas of the map are selected and grouped based on adjacency. These groups are filtered by size to remove noise. The centroids of each group are calculated and set as the destination. We use a modified A* algorithm combined with a recursive shadowcasting algorithm to efficiently determine a path to the destination. Once a path is completed the robot will only recompute the closest 5 points on the path as it travels. This allows the robot to update path trajectories in real time.

<img width="523" height="661" alt="image" src="https://github.com/user-attachments/assets/c9421315-73ca-4b7c-94a0-a828a79ddd94" />

![Video Project 5](https://github.com/user-attachments/assets/40a74386-2fed-4ab9-9ccb-dac3e39044db)

More detail about the implementation can be found in the attached report.
