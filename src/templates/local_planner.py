#!/usr/bin/env python

import os
import csv
import rospy
import numpy as np
from nav_msgs.msg import Odometry

# We will use KDTree to find  closest waypoint
from scipy.spatial import KDTree

# TODO: 1. Import required custom messages: BaseWaypoint, Waypoint, Path, LocalPath

# How many waypoints will be used for forward path 
MAX_WAYPOINTS = 10

# Which speed we want vehicle to drive with - m/s
PLANNED_SPEED = 10

class LocalPlanner(object):

    def __init__(self):
        # store KDTree built from mission waypoints 
        self.waypoints_xy = None
        
        # store last received vehicle position
        self.position = None

        # indicates that ROS master is initialized
        self.start_time = None
        
        # TODO: 2. Init local planner node 

        # wait ROS master is initialized
        self.wait_master_initialization()

        # TODO: 3. subscribe to /planner/mission_waypoints topic to get mission waypoints, please use init_mission callback
        
        # Subscribe Carla ROS bridge to get vehicle position and velocity
        rospy.Subscriber('/carla/ego_vehicle/odometry', Odometry, self.process_position)
               
        # Prepare publisher to publish  short-term path based on current vehicle position
        self.local_publisher = rospy.Publisher('/planner/local_waypoints', LocalPath, queue_size=1)

        # wait till we get planned path and vehicle position
        self.wait_initialization()

        self.loop()  
    
    def wait_master_initialization(self):
        rate = rospy.Rate(10)

        # wait till ROS master node is initialized 
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not rospy.is_shutdown():
            rospy.loginfo('Local Planner: ROS master initialized.')

    def wait_initialization(self):
        rate = rospy.Rate(10)

        # wait till we get position initialized 
        while not rospy.is_shutdown() and not self.position:
            rate.sleep()

        if not rospy.is_shutdown():
            rospy.loginfo('Local Planner: Connected to vehicle - got vehicle position.')

    def is_mission_initialized(self):
        return True if self.waypoints_xy else False

    def init_mission(self, path):
        # get waypoints from  message
        self.mission_waypoints = path.waypoints

        # We need to find waypoint from the path which is closest to the current car position
        # We can use simple search in the list but it have O(n) complexity which is generally slow
        # KD tree allows to search in Log(n) time https://en.wikipedia.org/wiki/K-d_tree
        # https://docs.scipy.org/doc/scipy-0.14.0/reference/generated/scipy.spatial.KDTree.html
        if not self.waypoints_xy:
            self.waypoints_xy = [[waypoint.x , waypoint.y] for waypoint in self.mission_waypoints]

            self.waypoint_tree = KDTree(self.waypoints_xy)
            rospy.loginfo('Local Planner: Received waypoints')

    def process_position(self, position):
        # TODO: 4. Store position received from car odometry topic as class field
        self.position = None
        pass

    def loop(self):
        # Re-plan forward path each 1/10 sec
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.is_mission_initialized() and self.position:
                self.plan_path()
     
            rate.sleep()

    def plan_path(self):
        # determine closest waypoint in the path going forward
        closest_waypoint_idx = self.get_car_closest_waypoint_idx()
        
        # prepare planned path forward
        local_path = self.prepare_local_path(closest_waypoint_idx)

        # TODO: 5. Publish further path to the /planner/local_waypoints topic
    
    # find current trajectory start waypoint starting from mission waypoint closest to the car
    def get_car_closest_waypoint_idx(self):
        # Structure of the Odometry message to correctly get x and y coordinates can be found there
        # http://docs.ros.org/melodic/api/nav_msgs/html/msg/Odometry.html
        x = self.position.pose.position.x
        y = self.position.pose.position.y
    
        # TODO: 6. Query KDTree to get closest waypoint        
       
        # TODO: 7. The problem is that closes waypoint can be behind car - we need to check this and keep next waypoint if this is the case  
        # We will use dot product for this purpose: http://www.leadinglesson.com/dot-product-is-positive-for-vectors-in-the-same-general-direction
        
        return closest_idx

    def prepare_local_path(self, closest_waypoint_idx):
        
        # find trajectory last index to handle waypoints in the end of the path
        farthest_idx = closest_waypoint_idx + MAX_WAYPOINTS
        last_index = len(self.waypoints_xy) - 1

        # select waypoints to publish as a short-term path starting from determined closest waypoint
        base_waypoints = self.mission_waypoints[closest_waypoint_idx:farthest_idx]

        # TODO: 8. Create new LocalPath object with selected waypoints, add desired speed to each endpoint (PLANNED_SPEED) 
    
        return path
     
if __name__ == '__main__':
    try:
        LocalPlanner()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start local planner node.')
        pass