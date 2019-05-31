#!/usr/bin/env python

import math
import rospy
from nav_msgs.msg import Odometry
import tf

#from pid import PID
#from stanley import Stanley

# TODO: 1.1 Import required custom message types to support control commands

# TODO: 2.1 Import required custom message types to support waypoints

class SimpleController(object):

    def __init__(self):
        # TODO: 1.2 Initialize ROS node
      
        # define state variables
        self.position = None
        self.x = None
        self.y = None
        self.v = None
        self.yaw = None
        self.start_time = None
        self.previous_time = None
        self.waypoints = None
        
        # wait ROS master is ready
        self.wait_master_initialization()

        # TODO: 2.2 Init controllers
        
        # TODO: 1.3 create publisher to publish CarlaVehicleControl to /carla/ego_vehicle/vehicle_control_cmd 
     
        # TODO: 2.3 Subscribe to local planner node to get trajectory with self.process_waypoints callback
         
        # TODO: 2.4 subscribe to Carla odometry to get current position and velocity with self.process_position callback
      
        # wait till we start getting odometry and waypoints from planner
        # TODO: 2.5 Uncomment code below so controller can be sure that we god position and waypoints
        #self.wait_initialization()
        
        # run node loop
        self.loop()

    def init_controllers(self):
        # Init PID Controller - set initial coefficients
        P = 0.8
        I = 0
        D = 0.3
        mn = 0.
        mx = 1.0
        
        self.throttle_controller = PID(P, I, D, mn, mx)

        # Init Stanley steering controller
        self.steering_controller = Stanley(1.22, 0.04)

    def process_position(self, position):
        self.position = position.pose
        
        # 2.6 we need to get current position: x, y, yaw angle from vehicle odometry message
        self.x = None
        self.y = None
       
        # Odometry message stored car heading using quaternions
        # https://www.youtube.com/watch?v=zjMuIxRvygQ
        # So we need to convert quaternions to euler "yaw" angle to determine current heading
        self.yaw = None
        
        #----------------------------------------------------------------------------------------

        v_x = position.twist.twist.linear.x
        v_y = position.twist.twist.linear.y

        # TODO: 2.7 We need to get car velocity from message. Message has x and y velocity components
        # we need to calculate total velocity based on x and y components  

        # This math could help: https://www.mathplanet.com/education/pre-algebra/right-triangles-and-algebra/the-pythagorean-theorem :) 
        self.v = None


    # method to convert quaternions to yaw angle
    def get_yaw_last_position(self):
        quaternion = (
                        self.position.pose.orientation.x,
                        self.position.pose.orientation.y,
                        self.position.pose.orientation.z,
                        self.position.pose.orientation.w)
            
        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw = euler[2]

        return yaw

    def process_waypoints(self, path):     
        # TODO: 2.8 Store last endpoints received in the class field
        pass

    def wait_master_initialization(self):
        rate = rospy.Rate(10)

        # wait till ROS master node is initialized 
        while not self.start_time and not rospy.is_shutdown():
            self.start_time = rospy.Time.now().to_nsec()

        if not rospy.is_shutdown():
            rospy.loginfo('Controller: ROS Master initialized.')

    def wait_initialization(self):
        rate = rospy.Rate(10)

        # wait till we get position initialized 
        while not rospy.is_shutdown() and not self.position:
            rate.sleep()

        if not rospy.is_shutdown():
            rospy.loginfo('Controller: Connected to vehicle - got vehicle position.')

        # wait till we get waypoints
        while not rospy.is_shutdown() and not self.waypoints:
            rate.sleep()
        
        if not rospy.is_shutdown():
            rospy.loginfo('Controller: Got waypoints')

    def loop(self):
        rate = rospy.Rate(10)
     
        while not rospy.is_shutdown():
            if not self.previous_time:
                # we need to track previous loop iteration time so we can calculate delta for PID controller
                self.previous_time = rospy.Time.now().to_nsec() / 1000.0
                throttle_output = 0
                steer_output    = 0
                brake_output = 0
            else:
                # get current time
                now = rospy.Time.now().to_nsec() / 1000.0
                
                # get time passed from previous command sent
                dt = now - self.previous_time
                self.previous_time = now

                # TODO: 1.4 Uncomment self.basic_control line for basic control
                #throttle_output, steer_output, brake_output =  self.basic_control()

                # TODO: 2.9 Uncomment self.control line for advanced control, comment self.basic_control() line above
                #throttle_output, steer_output, brake_output =  self.control(dt)

                # TODO: 1.5 publish controll messages to the /carla/ego_vehicle/vehicle_control_cmd topic to control vehicle

            rate.sleep()

    # basic controller to send constant velocity and steering to check connectivity
    def basic_control(self):
        return 0.3, 0.1, 0

    def control(self, dt):
        # WSL has issues with time - sometimes we do not have deference
        if dt == 0:
            dt = 0.001

        # get desired speed for next waypoint
        waypoints  = self.waypoints
        v_desired  = waypoints[0].v

        # Throttle controller section
        
        # calculate velocity error for PID controller error = desired - current
        vel_error = v_desired - self.v

        # call PID controller to get desired throttle/brake commands               
        throttle = self.throttle_controller.step(vel_error, dt)

        # need to brake if we have negative throttle
        if throttle > 0:
            throttle_output = throttle
            brake_output    = 0
        else:
            throttle_output =  0
            brake_output    = -throttle
            
        # Steering controller section
        
        # Determine heading error:
        #     1. Get trajectory points to determine current line approximation
        #     2. Determine trajectory heading direction - you can use: math.atan2 to find angle based on points
        #     3. Find difference between trajectory direction and car heading direction to calculate heading error

        # 1 Get trajectory points to determine current line approximation
        x1 = waypoints[0].x
        y1 = waypoints[0].y
        
        x2 = waypoints[1].x
        y2 = waypoints[1].y        
      
        # 2  Determine trajectory heading direction - you can use: math.atan2 to find angle 
        delta_x = x2 - x1
        delta_y = y2 - y1
        theta_radians = math.atan2(delta_y, delta_x)

        # 3  Determine trajectory heading direction - you can use: math.atan2 to find angle 
        diff = theta_radians - self.yaw 

        if diff > math.pi:
            diff = diff - 2 * math.pi 

        if diff < -math.pi:
            diff = diff + 2 * math.pi 

        # Determine cross-track error e:
        #     We need to find distance from vehicle(point) to the trajectory line (line)
        #     https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line see "Line defined by two points" section

        e = ((y2 - y1) * self.x  - (x2 - x1) * self.y + x2 * y1 - y2 * x1) / math.sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2))

        # call Stanley controller to minimize heading and cross-track error
        steer = -self.steering_controller.step(diff, self.v, e)

        steer_output  = steer       
        
        return throttle_output, steer_output, 0
 
if __name__ == '__main__':
    try:
        SimpleController()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start mission planner node.')
        pass