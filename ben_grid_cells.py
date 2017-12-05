#!/usr/bin/env python

import rospy
from nav_msgs.msg import GridCells
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Point, Pose, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry, OccupancyGrid, Path
from kobuki_msgs.msg import BumperEvent
import tf
import numpy
import math 
import rospy, tf, numpy, math
from _hotshot import resolution
from OpenGL.GL.EXT import coordinate_frame
import heapq
import copy
import numpy


def getGridVal(coord):
    (x, y) = coord
    return mapData[y * width + x]

def checkIfFullGrid(coord):
    #takes a xy tuple coordinate and returns true if 
    #the value is in the list of occupied cells
    return coord in fullGrid

def checkIfSpaceList(coord):
    #takes a xy tuple coordinate and returns true if 
    #the value is in the list of occupied cells
    return coord in spaceList

def checkIfOccupied(coord):
    #takes a xy tuple coordinate and returns true if 
    #the value is in the list of occupied cells
    return coord in occupiedList

def checkIfExtended(coord):
    #takes a xy tuple coordinate and returns true if 
    #the value is in the list of occupied cells
    return coord in extendedList

def getPathWaypoints(list_of_xy):
    path_waypoints = Path()
    
    #add the start pose to the path
    path_waypoints.header.frame_id = 'map'
    path_waypoints.poses.append(getCurrentLocation())
    
    current = list_of_xy[0]
    next = list_of_xy[1]
    old_diff = tuple(numpy.subtract(next, current))
    for index, xy in enumerate(list_of_xy[1:-1]):
        current = xy
        next = list_of_xy[index+2]
        new_diff = tuple(numpy.subtract(next, current))
        
        #for adding the "joints" between straight path segments
        if new_diff != old_diff:
            old_diff = new_diff
            wp = PoseStamped()
            wp.header.frame_id = 'map'
            wp.pose.position.x = current[0] * resolution
            wp.pose.position.y = current[1] * resolution
            wp.pose.orientation.w = 1.0
            
            #TODO: calc orientation for current-to-next by using (previous, current(joint point) and next)
            
            path_waypoints.poses.append(wp)
        
        #for adding goal gridsquare as final waypoint
        if next == list_of_xy[-1]:
            end_wp = PoseStamped()
            end_wp.header.frame_id = 'map'
            end_wp.pose.position.x = next[0] * resolution
            end_wp.pose.position.y = next[1] * resolution
            end_wp.pose.orientation.w = 1.0
            path_waypoints.poses.append(end_wp)
    
    return path_waypoints

def cleanGridCells():
    path_pub.publish(createGridCells([]))
    expanded_pub.publish(createGridCells([]))
    frontier_pub.publish(createGridCells([]))
    waypoint_pub.publish(createBlankPath([]))

def getNeighbors(coord, radius):
    (x, y) = coord
    neighbors = []
    i=y-radius
    j=x-radius
    for j in range(x-radius,x+radius):
        for i in range (y-radius,y+radius):
            tup=(j,i)
            neighbors.append(tup)
            i=i+1
        j=j+1

    if (x + y) % 2 == 0: neighbors.reverse()  # aesthetics
	
    #check if within bounds
    neighbors = list(filter(lambda xy: 0 <= xy[0] < width and 0 <= xy[1] < height, neighbors))  # within bounds of grid
    
    return neighbors
    # grid value is 0 (passable)
    return neighbors

			

def getCurrentLocation():
    tf_listener.waitForTransform('map', 'base_link', rospy.Time(0), rospy.Duration(1.0))
    (position, orientation) = tf_listener.lookupTransform('map', 'base_link', rospy.Time(0))
    # finds the position and oriention of two objects relative to each other (hint: this returns arrays, while Pose uses lists)
    
    rospy.logerr("Current XY: %f %f", position[0], position[1])

    current = PoseStamped()
    current.header.frame_id = 'map'
    current.pose.position.x = position[0]
    current.pose.position.y = position[1]

    current.pose.orientation.x = orientation[0]
    current.pose.orientation.y = orientation[1]
    current.pose.orientation.z = orientation[2]
    current.pose.orientation.w = orientation[3]
    
    return current

class PriorityQueue:
    def __init__(self):
        self.elements = []
    
    def empty(self):
        return len(self.elements) == 0
    
    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))
    
    def get(self):
        return heapq.heappop(self.elements)[1]
    
def reconstruct_path(came_from, start, goal, visual_mode=True):
    current = goal
    path = []
    while current != start:
        path.append(current)
        if visual_mode:
            rospy.sleep(0.001)
            path_pub.publish(createGridCells(path))
        current = came_from[current]
    path.append(start) # optional
    path.reverse() # optional
    path_pub.publish(createGridCells(path))
    return path

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def cost(a, b):
    return 1

# reads in global map
def mapCallBack(data):
    global mapData
    global width
    global height
    global mapgrid
    global resolution
    global offsetX
    global offsetY
    
    #global extWallData
    #i kinda just threw this here i know its a sloppy place to put it but oh well RBE code lul

    mapgrid = data
    resolution = data.info.resolution
    mapData = data.data
    width = data.info.width
    height = data.info.height
    offsetX = data.info.origin.position.x
    offsetY = data.info.origin.position.y
    print data.info
    
def readGoal(goal):
    global goalX
    global goalY
    goalX= goal.pose.position.x
    goalY= goal.pose.position.y
    
    cleanGridCells()
    
    rospy.logwarn("Offset XY: %f %f", offsetX, offsetY)
    rospy.logwarn("CLICK XY: %f %f", goal.pose.position.x, goal.pose.position.x)
    rospy.logwarn("CLICK GRID XY: %d %d", int(goal.pose.position.x / resolution), int(goal.pose.position.y / resolution))
    
    current = getCurrentLocation()
    start_xy = (int(current.pose.position.x / resolution), int(current.pose.position.y / resolution))
    goal_xy = (int(goalX / resolution), int(goalY /resolution))
    
    rospy.logwarn("START XY: %s", start_xy)
    rospy.logwarn("GOAL XY: %s", goal_xy)

    if getGridVal(goal_xy) != 0:
	rospy.logerr("Goal is in an obstacle, cannot find path")

    else:
    
        (came_from, cost_so_far) = aStar(start_xy, goal_xy, True)
    
        print "\n\n\n"
    
        path = reconstruct_path(came_from, start_xy, goal_xy, 0.5)
    
        print "Path: ",path
    
        path_poses = getPathWaypoints(path)
        waypoint_pub.publish(path_poses)

#def pathMarkers():
	

def aStar(start, goal, visual_mode=False):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    
    if visual_mode:
        rospy.sleep(0.001)
        frontier_pub.publish(createGridCells(x[1] for x in frontier.elements))
        
    came_from = {}
    cost_so_far = {}
    came_from[start] = None
    cost_so_far[start] = 0
    
    while not frontier.empty():
        current = frontier.get()
        neighbors = getNeighbors(current,2)
        neighbors = list(filter(lambda xy: checkIfExtended(xy)==False, neighbors))
        if current == goal:
            rospy.logerr("ASTAR FOUND GOAL!!!")
            break
        
        print (neighbors)
        for next in neighbors:
            new_cost = cost_so_far[current] + cost(current, next) #could use hueristic instead of cost but it seems to be awful
            
            if next not in cost_so_far or new_cost < cost_so_far[next]:
                cost_so_far[next] = new_cost
                priority = new_cost + heuristic(goal, next)
                frontier.put(next, priority)
                came_from[next] = current
                
                if visual_mode:
                    rospy.sleep(0.001)
                    frontier_pub.publish(createGridCells(x[1] for x in frontier.elements))
                    expanded_pub.publish(createGridCells(came_from))
                    
    return came_from, cost_so_far

def publishCells(grid):
    global pub

    #update map
    pub.publish(createGridCells(getSpaceList()))
    #extend the occupied cells to add the width of the robot 
    occupiedExt_pub.publish(createGridCells(getExtendedList()))
    
def createGridCells(list_of_xy):
    #Takes in teh GRID x and y values 
    #  and makes a GridCells() object that has the actual coordinates 
    #(for displaying)

    #setup a GridCells object
    cells = GridCells()
    cells.header.frame_id = '/map'
    cells.cell_width = resolution 
    cells.cell_height = resolution

    #add the inputed list of xy values 
    alignXYtoGridCell(list_of_xy,cells)
    
    #return the cells
    return cells

def alignXYtoGridCell(list_of_xy, cells):
   #takes in a list of x and y values (in a tuple) and a GridCells object
   # and returns the full GridCells object with teh points offset and added
   for (x, y) in list_of_xy:
        point=Point()
        point.x = (x * resolution) + offsetX + (.5 * resolution)
        point.y = (y * resolution) + offsetY + (.5 * resolution)
        point.z = 0
        cells.cells.append(point)
   return cells

def getFullGrid():
    xyList = []
    #here we will iterate over the entire height and width of the set
    # adds an x, y value for each of these points to a list 
    #takes no input and returns a list of x and y tuples and 
    for i in range(0,height-1): #height should be set to hieght of grid
        for j in range(0,width-1): #width should be set to width of grid
           tup = (j,i)
           #tuple of the j, i values
           xyList.append(tup)
           #add it to the final list
    return xyList

def getSpaceList():
    #returns the list of xy that arent occupied
    xyList=[]
    k=0
    #here we will iterate over the entire height and width of the set
    # adds an x, y value for each of these points to a list 
    for i in range(0,height-1): #height should be set to hieght of grid
        for j in range(0,width-1): #width should be set to width of grid
            #print k # used for debugging
            if (mapData[k] == 0):
		
                tup = (j,i)
		#tuple of the j, i values
                xyList.append(tup)

            k=k+1
        k=k+1
    return xyList

def getOccupiedList():
     #gets list of occupied xy values
     #here we will iterate over the entire height and width of the set
     #adds an x, y value for each of these points to a list 
     k=0
     xyList=[]
     for i in range(0,height-1): #height should be set to hieght of grid
        for j in range(0,width-1): #width should be set to width of grid
            #print k # used for debugging
            if (mapData[k] != 0):
		
                tup = (j,i)
		#tuple of the j, i values
                xyList.append(tup)

            k=k+1
        k=k+1
     return xyList

def getExtendedList():
    k=0
    xyList=[]
    for i in range(0,height-1): #height should be set to hieght of grid
       for j in range(0,width-1): #width should be set to width of grid
           #print k # used for debugging
           if (mapData[k] != 0):
              coord = (j,i)
	      for xyIt in getNeighbors(coord,5):
	      	xyList.append(xyIt)
           k=k+1
       k=k+1
    return xyList

def setupDataSets():
    global extendedList 
    global occupiedList 
    global spaceList 
    global fullGrid 
    extendedList = getExtendedList()
    occupiedList = getOccupiedList()
    spaceList = getSpaceList()
    fullGrid = getFullGrid()


def createBlankPath(list_of_xy):
   path = Path()
   path.header.frame_id = '/map'
   return path

#Main handler of the project
def run():
    global pub
    global path_pub
    global expanded_pub
    global frontier_pub
    
    global occupiedExt_pub
    
    global waypoint_pub
    
    global pathData
    pathData = []
    
    global tf_listener
    tf_listener = tf.TransformListener()
    
    
    rospy.init_node('lab3')
    sub = rospy.Subscriber("/map", OccupancyGrid, mapCallBack)
    pub = rospy.Publisher("/map_check", GridCells, queue_size=1)  
    path_pub = rospy.Publisher("/path", GridCells, queue_size=1)
    expanded_pub = rospy.Publisher("/expand", GridCells, queue_size=1)
    frontier_pub = rospy.Publisher("/frontier", GridCells, queue_size=1)
    waypoint_pub = rospy.Publisher("/waypoints", Path, queue_size=1)
    goal_sub = rospy.Subscriber('move_base_astar/goal', PoseStamped, readGoal, queue_size=1)
    occupiedExt_pub = rospy.Publisher("/occext", GridCells, queue_size=1)
 #change topic for best results

    # wait a second for publisher, subscribers, and TF
    rospy.sleep(1)

    setupDataSets()

    while (1 and not rospy.is_shutdown()):
        publishCells(mapData) #publishing map data every 2 seconds
        rospy.sleep(4)  #TODO CHANGE BACK TO 2

if __name__ == '__main__':
    try:
        run()
    except rospy.ROSInterruptException:
        pass2
