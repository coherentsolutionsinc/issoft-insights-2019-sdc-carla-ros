#!/usr/bin/env python

import os
import csv
import rospy

# TODO: 1. Import waypoint messages
from sdc_package.msg import BaseWaypoint, Path

class MissionPlanner(object):

    def __init__(self):
        self.start_time = None

        # TODO: 2. Init mission planner node
        rospy.init_node('mission_planner')
        #-------------------------------------------------------------------------------------------------------
        
        self.wait_master_initialization()
        
        # TODO: 3. Create publisher to publish mission path to /planner/mission_waypoints topic
        self.waypoints_publisher = rospy.Publisher('/planner/mission_waypoints',
                                Path, queue_size=1, latch=True)
        #-------------------------------------------------------------------------------------------------------
        
        # TODO: 4. Get waypoints file path from parameters
        waypoints_file_path = rospy.get_param('~waypoints_path')
        #-------------------------------------------------------------------------------------------------------
        

        #TODO 5. Load waypoints from fie using "load_waypoints" method
        waypoints = self.load_waypoints(waypoints_file_path)
        #-------------------------------------------------------------------------------------------------------
        
        # TODO: 6. Publish waypoints to created publisher
        self.publish_waypoints(waypoints)
        #-------------------------------------------------------------------------------------------------------
                
        rospy.loginfo('Waypoints published')
        
        # TODO: 7. Run Empty ROS loop to keep node online using rospy.spin()
        rospy.spin()
        #-------------------------------------------------------------------------------------------------------
        
    # Wait ROS Master node is initialized 
    def wait_master_initialization(self):
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not rospy.is_shutdown():
            rospy.loginfo('Mission Planner: ROS master initialized.')
 

    def load_waypoints(self, path):
        waypoints = []

        # check if file path is valid 
        if os.path.isfile(path):
            waypointsFile = open(path, 'r')  
           
            # read csv file
            with waypointsFile:  
                reader = csv.reader(waypointsFile)
                for row in reader:
                    # row[0] to access first csv element
                    # row[1] to access second csv element
                    
                    # TODO: 8. Create new BaseWaypoint object and add to waypoints array
                    waypoint = BaseWaypoint()
                    waypoint.x = float(row[0])
                    waypoint.y = -float(row[1])
                    waypoints.append(waypoint)
                    #-------------------------------------------------------------------------------------------------------
        
            rospy.loginfo('Waypoints Loaded: found %d waypoints', len(waypoints))
        else:
            rospy.logerr('%s is not a file', path)
        
        return waypoints

    def publish_waypoints(self, waypoints):

        # TODO: 9. Crete new Path message with waypoints provided and publish to /planner/mission_waypoints topic
        path = Path()  
        path.waypoints = waypoints
        #-------------------------------------------------------------------------------------------------------
        self.waypoints_publisher.publish(path)      

if __name__ == '__main__':
    try:
        MissionPlanner()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start Mission Planner node.')
        pass