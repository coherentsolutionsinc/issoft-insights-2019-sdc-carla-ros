#!/usr/bin/env python

import os
import csv
import rospy

# class to host node logic
class NodeClass(object):

    def __init__(self):
        
        # initialize node with unique name
        rospy.init_node('node_name')
        
        # sample of the publisher
        self.publisher1 = rospy.Publisher('<topic name>',
                                <message_type>, queue_size=1, latch=True)

        # <topic name>' name of the topic to subscribe
        # <message_type> type of the message from topic
        # queue_size number of last messages to keep in queue if not processed
        # latch indicates if we need to keep messages if no any subscribers


        # sample subscriber
        rospy.Subscriber('<topic name>', <message_type>, self.callback_function)

        # sample of using param server to get configuration parameter
        waypoints_file_path = rospy.get_param('~waypoints_path')

        # Way to keep node running if we do not have any logic to be executed in node loop        
        rospy.spin()

        # or 

        # Way to keep node running with logic in the loop
        self.loop()


    # main node loop 
    def loop(self):
        
        # define loop rate in Hz
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            # DO some logic

            # wait 1/50 sec
            rate.sleep()


if __name__ == '__main__':
    try:
        NodeClass()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start <template> node.')
        pass