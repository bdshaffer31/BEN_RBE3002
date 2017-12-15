Synopsis
========
This project was to design software to run on the Turtlebot that would autonomously explore the frontier and map an enclosed area.

Description
========
In general, the purpose of the project was to build an informed search algorithm on a grid, so that the robot could explore the environment. 
The robot had to be able to locate borders of the unexplored zones (shown in orange) and find a path to those borders using an A* search. After that, the goal was to drive to the borders in order to explore those zones by spinning in one place.

The project is interesting from the software engineering stand-point because it is very high-level (no low-level robotics involved), allowing to practice search algorithms, such as BFS, DFS and A*, and performance optimization techniques, such as multi-threading.  


Directions
============

Clone repo and add to workspace
catkin_ws/src/

'roscore' to start a new ros core

'roslaunch turtlebot_gazebo turtlebot_world.launch'to bring up simulation or use actual turtlebot.

'rosrun rviz rviz' to open rviz, and then open the config from rviz folder

Then run the following commands:
1) 'roslaunch try_again final_project.launch'
2) 'rosrun try_again mapping.py'
3) 'rosrun try_again control.py'

save the map

