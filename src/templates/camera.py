# TODO: 1. Add indicator that node should be run by python

# line above indicates that python is responsible for running this node

import os
import csv
import rospy
import numpy as np
import pygame

from utilities import pipline

import cv2
from cv_bridge import CvBridge, CvBridgeError

from sensor_msgs.msg import Image

# set image resolution
RESOLUTION_X = 640
RESOLUTION_Y = 480

# python class definition
class CameraTester(object):

    # python constructor definition
    def __init__(self):
        self.start_time = None
        self.image  = None
        self.got_image = False
        self.init_pygame()
        self.bridge = CvBridge()

        # TODO: 2. Init node - give node an unique name - overwritten from launch file

        # wait master node is initialized and record start time
        self.wait_master_initialization()
        
        # TODO: 3. Subscribe to the ROS bridge camera topic and provide callback

        # wait till we got first image
        self.wait_initialization()

        # run node infinite loop      
        self.loop()  
    
    # TODO: 4. Write callback method for the subscriber

    # init pygame window to display images
    def init_pygame(self):
        pygame.init()
        pygame.display.set_caption("Camera images")
        self.screen = pygame.display.set_mode([RESOLUTION_X, RESOLUTION_Y])

    # wait master node is initialized and record start time
    def wait_master_initialization(self):
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not rospy.is_shutdown():
            rospy.loginfo('CameraTester: Ros master initialized.')
   
    # wait till we got first image
    def wait_initialization(self):
        # define sleep rate foe the loop
        rate = rospy.Rate(10)

        # wait till we get image initialized 
        while not rospy.is_shutdown() and not self.got_image:
            rate.sleep()

        if not rospy.is_shutdown():
            rospy.loginfo('CameraTester: Connected to vehicle - got camera images')

    # main node loop 
    def loop(self):
        
        # define loop rate in Hz
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.image is not None:
                # process stored image and display it in pygame window
                self.process_frame()
                # update pygame window
                pygame.display.flip()

            # wait 1/20 sec 
            rate.sleep()

    # convert open cv image to pygame image and display
    def process_frame(self):
        # we need to convert image as it use BGR color scheme and flipped 
        frame = cv2.cvtColor(self.image, cv2.COLOR_BGR2RGB)
        frame = np.rot90(frame)
        frame = np.flip(frame, 0)
        
        # TODO: 5. Add sample image processing - for example filters useful to lanes detection - uncomment line below    
        #frame = pipline(frame)     
       
        frame = pygame.surfarray.make_surface(frame)
        self.screen.blit(frame,(0,0))
        return

# python way to indicate what to do if this file is run as executable rather then imported as library
if __name__ == '__main__':
    try:
        # create CameraTester instance and initiate loop sequence
        CameraTester()
    except rospy.ROSInterruptException:
        # catch and log ROS errors
        rospy.logerr('Could not start camera tester node.')
        pass
    finally:
        pygame.quit()