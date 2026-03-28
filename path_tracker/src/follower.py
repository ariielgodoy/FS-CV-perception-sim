#!/usr/bin/env python2

import rospy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Joy
from ackermann_msgs.msg import AckermannDriveStamped
import math
import numpy as np


class Drive:

    def __init__(self):

        ## PARAMETERS ##

        self.mode = 4   # Initial operation mode (do nothing)

        self.L_wall = 0.8   # Look-ahead (m) for Wall Following
        self.L_cor = 0.8    # Look-ahead (m) for Corridor Following
        self.dw = 1.0       # Distance to the wall (m)
        self.v = 0.8        # Linear velocity (m/s)

        # SIMULATED CAR
        self.dy = 0.265             # Distance between the center of the robot and the center of the laser (m)
        self.l = 0.32               # Wheelbase between the front and rear wheels (m)
        self.psi = 18*(np.pi/180)   # Maximum steering angle of the wheels

        # REAL CAR
        # self.dy = 0.315
        # self.l = 0.315
        # self.psi = 18*(np.pi/180)

        # SAFE ZONE
        self.rmin = self.l / (math.tan(self.psi))           # Minimum steering radius
        # print("rmin", self.rmin)
        self.sz = self.dw + self.rmin                       # Safe distance for wall following
        self.wmax = self.v * (math.tan(self.psi) / self.l)  # Maximum angular velocity for minimum steering angle

        # PERSON FOLLOWER
        self.ds = 0.8               # Initial search distance for the person
        self.dth = 0.54             # Threshold
        self.df = 3.0               # Maximum distance
        self.detected = False       # Check if the person has been detected
        self.init_angle = False     # Initial angle of the detected person
        self.x_k = 0                # Initial x-coordinate of the detected person
        self.y_k = 0                # Initial y-coordinate of the detected person
        self.rho_k = 0
        self.theta_k = 0
        
        ## TOPICS ##

        laser_topic = '/scan'
        joy_topic = '/vesc/joy'
        drive_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'

        self.sent_message = AckermannDriveStamped()

        ## SUBSCRIPTIONS ##

        rospy.Subscriber(joy_topic, Joy, self.joy_callback)
        rospy.Subscriber(laser_topic, LaserScan, self.scan_callback)

        ## PUBLICATIONS ##

        self.pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=1)

        rospy.spin()


    def joy_callback(self, data):

        ## SIMULATION ##

        if data.buttons[0]:
            self.mode = 0
            print("X button pressed, running: person")

        elif data.buttons[1]:
            self.detected = False
            self.mode = 1
            print("O button pressed, running: rigth wall")

        elif data.buttons[2]:
            self.detected = False
            self.mode = 2
            print("D button pressed, running: corridor")

        elif data.buttons[3]:
            self.detected = False
            self.mode = 3
            print("[] button pressed, running: left wall")

        ## TESTING ##

        # if data.buttons[14]:
        #     self.mode = 0
        #     print("X button pressed, running: person")

        # elif data.buttons[13]:
        #     self.detected = False
        #     self.mode = 1
        #     print("O button pressed, running: rigth wall")

        # elif data.buttons[12]:
        #     self.detected = False
        #     self.mode = 2
        #     print("D button pressed, running: corridor")

        # elif data.buttons[15]:
        #     self.detected = False
        #     self.mode = 3
        #     print("[] button pressed, running: left wall")

        return self.mode


    def left_wall(self, msg):

        ## SCANNER DATA ##

        n_ranges = self.ranges.size
        index_min = np.argmin(self.ranges[int(n_ranges/2):n_ranges]) + int(n_ranges/2)
        rho = self.ranges[index_min]        
        angle = self.angles[index_min]
        sign_angle = math.copysign(1, angle)

        print("LEFT_WALL - distance = {}, angle = {}".format(rho, angle*180/3.14))
        
        ## SAFE ZONE ##

        safe = self.ranges[int(n_ranges/2)] 

        if safe < self.sz:
            w = - self.wmax
            print("LEFT_WALL - safe zone")

        else:

            ## PURE-PURSUIT ALGORITHM ##

            yp = sign_angle * (rho + self.dy * math.cos(angle) - self.dw)

            if abs(yp) < self.L_wall:
                ygg = yp
                xgg = math.sqrt(self.L_wall**2 - yp**2)

            else:
                ygg = self.L_wall * math.copysign(1, yp)
                xgg = 0

            yg = sign_angle * (ygg * math.sin(angle) - xgg * math.cos(angle))
            gamma = (2 * yg) / (self.L_wall**2)
            w = gamma * self.v

        ## ACKERMANN CONVERSION ##

        steering_angle = math.atan((w * self.l) / self.v)

        return steering_angle


    def rigth_wall(self, msg):

        ## SCANNER DATA ##

        n_ranges = self.ranges.size
        index_min = np.argmin(self.ranges[0:int(n_ranges/2)])
        rho = self.ranges[index_min]
        angle = self.angles[index_min]
        sign_angle = math.copysign(1, angle)

        print("RIGTH_WALL - distance = {}, angle = {}".format(rho, angle*180/3.14))
        
        ## SAFE ZONE ##

        safe = self.ranges[int(n_ranges/2)] 

        if safe < self.sz:
            w = self.wmax
            print("RIGTH_WALL - safe zone")

        else:

            ## PURE-PURSUIT ALGORITHM ##

            yp = sign_angle * (rho + self.dy * math.cos(angle) - self.dw)

            if abs(yp) < self.L_wall:
                ygg = yp
                xgg = math.sqrt(self.L_wall**2 - yp**2)

            else:
                ygg = self.L_wall * math.copysign(1, yp)
                xgg = 0

            yg = sign_angle * (ygg * math.sin(angle) - xgg * math.cos(angle))
            gamma = (2 * yg) / (self.L_wall**2)
            w = gamma * self.v

        ## ACKERMANN CONVERSION ##

        steering_angle = math.atan((w * self.l) / self.v)

        return steering_angle
    

    def corridor(self, msg):

        ## SCANNER DATA ##

        n_ranges = self.ranges.size

        min_right = np.argmin(self.ranges[0:int(n_ranges/2)])                               # Index min from -135 to 0 grades
        min_left = np.argmin(self.ranges[int(n_ranges/2+1):n_ranges]) + int(n_ranges/2+1)   # Index min from 0 to +135 grades

        rho_right = self.ranges[min_right]
        rho_left = self.ranges[min_left]

        theta_right = self.angles[min_right]
        theta_left = self.angles[min_left]
        theta_c = theta_right + theta_left

        print("CORRIDOR - dist_right = {}, angle_right = {}".format(rho_right, theta_right*np.pi/180))
        print("CORRIDOR - dist_left  = {}, angle_left  = {}".format(rho_left, theta_left*np.pi/180))

        ylg = (rho_left) + (self.dy * math.cos(theta_left))
        yrg = -(rho_right) - (self.dy * math.cos(theta_right))
        ypg = (ylg + yrg) / 2

        ## SAFE ZONE ##

        safe = self.ranges[int(n_ranges/2)] 
        sz = ((rho_right + rho_left) / 2) + self.rmin

        if safe < sz:

            if rho_left > rho_right:        # Like right wall following
                w = self.wmax

            elif rho_left < rho_right:      # Like left wall following
                w = - self.wmax
            
            print("CORRIDOR - safe zone")

        else:

            ## PURE-PURSUIT ALGORITHM ##

            if abs(ypg) < self.L_cor:
                ygg = ypg
                xgg = math.sqrt(self.L_cor**2 - ypg**2)

            else:
                ygg = self.L_cor * math.copysign(1, ypg)
                xgg = 0

            yg = xgg * math.sin(theta_c) + ygg * math.cos(theta_c)
            gamma = (2 * yg) / (self.L_cor**2)
            w = gamma * self.v

        ## ACKERMANN CONVERSION ##

        steering_angle = math.atan((w * self.l) / self.v)

        return steering_angle


    def person(self, msg):
        
        ## SCANNER DATA ##

        n_ranges = self.ranges.size
        
        if self.detected == False:

            self.rho_k = self.ranges[int(n_ranges/2)]
            print("PERSON - first distance detected = ", self.rho_k)
            self.theta_k = (msg.angle_min + msg.angle_max) / 2

            if (self.rho_k >= (self.ds - self.dth)) and (self.rho_k <= (self.ds + self.dth)):
                self.detected = True
                self.x_k = self.dy + self.rho_k * math.cos(self.theta_k) 
                self.y_k = self.rho_k * math.sin(self.theta_k)

            else:
                steering_angle, vsp = (0, 0)
                
        elif self.detected == True:

            x_scan = self.ranges * np.cos(self.angles)
            y_scan = self.ranges * np.sin(self.angles)
            d_k = np.sqrt(((x_scan + self.dy) - self.x_k)**2 + (y_scan - self.y_k)**2)
            d_k_min = np.min(d_k)
            
            ## PURE-PURSUIT ALGORITHM ##

            yg = self.rho_k * math.sin(self.theta_k)
            L = math.sqrt(self.rho_k * (2 * self.dy * math.cos(self.theta_k) + self.rho_k))
            gamma = (2 * yg) / (L**2)

            if self.rho_k < self.ds:
                vsp = 0

            elif self.rho_k >= self.ds and self.rho_k <= self.df:
                vsp = ((self.rho_k - self.ds) / (self.df - self.ds)) * self.v
            
            if d_k_min <= self.dth:
                k_min = np.argmin(d_k)
                self.x_k = x_scan[k_min]
                self.y_k = y_scan[k_min]
                self.rho_k = self.ranges[k_min]
                self.theta_k = self.angles[k_min]

            else:
                self.detected = False
                steering_angle, vsp = (0, 0)

        if self.detected == True:

            yg = self.rho_k * math.sin(self.theta_k)
            L = math.sqrt(self.rho_k * (2 * self.dy * math.cos(self.theta_k) + self.rho_k))
            gamma = (2 * yg) / (L**2)

            ## lINEAR VELOCITY ##

            if self.rho_k < self.ds:
                vsp = 0

            elif self.rho_k >= self.ds and self.rho_k <= self.df:
                vsp = ((self.rho_k - self.ds) / (self.df - self.ds)) * self.v

            elif self.rho_k > self.df:
                vsp = self.v * 1.3              
    
            w = gamma * vsp

            ## ACKERMANN CONVERSION ##

            steering_angle = math.atan2(w * self.l, vsp)

            print("PERSON - x = {}, y = {}, dist = {}, vel = {}".format(self.x_k, self.y_k, self.rho_k, vsp))

        return steering_angle, vsp


    def scan_callback(self, msg):
        
        if self.init_angle == False:
            self.angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
            self.init_angle = True
            
        self.ranges = np.nan_to_num(np.array(msg.ranges))
        self.ranges[self.ranges < 0.0011] = 1e6

        ## BREAK ZONE ##

        if np.min(self.ranges) < 0.25:   
            steering_angle = 0
            v = 0

        else:

            ## Steering angle and linear velocity depending on path tracking mode ##
            
            if self.mode == 0:
                steering_angle, v = Drive.person(self, msg)

            elif self.mode == 1:
                steering_angle = Drive.rigth_wall(self, msg)
                v = self.v

            elif self.mode == 2:
                steering_angle = Drive.corridor(self, msg)
                v = self.v

            elif self.mode == 3:
                steering_angle = Drive.left_wall(self, msg)
                v = self.v

            else:
                steering_angle = 0
                v = 0

        ## PUBLISH SPEED & STEERING ANGLE ##

        if steering_angle > self.psi:
            steering_angle = self.psi

        elif steering_angle < -self.psi:
            steering_angle = -self.psi        

        self.sent_message.drive.speed = v
        self.sent_message.drive.steering_angle = steering_angle         # Simulation
        # self.sent_message.drive.steering_angle = - steering_angle     # Testing
        self.pub.publish(self.sent_message)
    

if __name__ == '__main__':
    rospy.init_node('follower')
    Drive()
    rospy.spin()