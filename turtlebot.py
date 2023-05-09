#!/usr/bin/env python

import time
import rospy
import math
import pandas as pd
import numpy as np
from geometry_msgs.msg import Twist,PoseStamped
from nav_msgs.msg import Odometry,Path
from array import *
import tf

import os
import rospy
import pickle

global x,y
class Test1():
    def __init__(self):

        self.goal_x = 3
        self.goal_y = 6
        self.goal_theta = -math.pi/2

        #### to change
        self.init_x = 0
        self.init_y = 0
        self.init_theta = 0
        #################

        rospy.init_node('Test1', anonymous=False)
        rospy.on_shutdown(self.shutdown)
        
        # do not change
        self.world_x = 0
        self.world_y = 0
        self.world_theta = 0
        self.x, self.y, self.theta = 0, 0, 0 # self coordinate        
        self.cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.callback)

        move_cmd = Twist()

        s = [self.init_x, self.init_y, self.init_theta] 
        # initial action 

        # set initial action to move towards goal
        a = self.get_next_action(s, [self.goal_x, self.goal_y, self.goal_theta])
        move_cmd.linear.x = a[0]
        move_cmd.angular.z = a[1]

        rate = rospy.Rate(10)
        start = time.time()
        init_time = time.time()

        while not rospy.is_shutdown():
            # publish speed to robot
            self.cmd_vel.publish(move_cmd)

            # check if goal has been reached
            if self.is_goal_reached():
                print("Goal reached!")
                self.cmd_vel.publish(Twist())  # stop the robot
                break

            # get next action
            s = [self.world_x, self.world_y, self.world_theta]
            a = self.get_next_action(s, [self.goal_x, self.goal_y, self.goal_theta])

            move_cmd.linear.x = a[0]
            move_cmd.angular.z = a[1]

            # what to do every 0.1 s
            self.world_x = self.x*math.cos(self.init_theta) - self.y*math.sin(self.init_theta) + self.init_x 
            self.world_y = self.x*math.sin(self.init_theta) + self.y*math.cos(self.init_theta) + self.init_y
            self.world_theta = self.theta + self.init_theta

            print(self.world_x, self.world_y, self.world_theta)

            rate.sleep()

    def is_goal_reached(self):
        # check if robot has reached the goal
        return abs(self.world_x - self.goal_x) < 0.1 and abs(self.world_y - self.goal_y) < 0.1 and abs(self.world_theta - self.goal_theta) < 0.1

    # def get_next_action(self, s, goal):
    #     # calculate next action based on current state and goal
    #     x, y, theta = s
    #     goal_x, goal_y, goal_theta = goal

    #     # calculate distance and angle to goal
    #     dx = goal_x - x
    #     dy = goal_y - y
    #     goal_dist = math.sqrt(dx**2 + dy**2)
    #     goal_angle = math.atan2(dy, dx)

    #     # calculate angular velocity
    #     angle_error = goal_angle - theta
    #     while angle_error > math.pi:
    #         angle_error -= 2 * math.pi
    #     while angle_error < -math.pi:
    #         angle_error += 2 * math.pi
    #     angular_vel = angle_error

    #     # calculate linear velocity
    #     linear_vel = 0.5 * goal_dist

    #     # limit linear and angular velocities
    #     if linear_vel > 0.5:
    #         linear_vel = 0.5
    #     elif linear_vel < -0.5:
    #         linear_vel = -0.5
    #     if angular_vel > 1.0:
    #         angular_vel = 1.0
    #     elif angular_vel < -1.0:
    #         angular_vel = -1.0

    #     return [linear_vel, angular_vel]
        
    def get_next_action(self, s, goal):
        # calculate next action based on current state and goal
        x, y, theta = s
        goal_x, goal_y, goal_theta = goal

        # calculate distance and angle to goal
        dx = goal_x - x
        dy = goal_y - y
        goal_dist = math.sqrt(dx**2 + dy**2)
        goal_angle = math.atan2(dy, dx)

        # calculate angular velocity
        angle_error = goal_angle - theta
        while angle_error > math.pi:
            angle_error -= 2 * math.pi
        while angle_error < -math.pi:
            angle_error += 2 * math.pi
        if abs(angle_error) > 0.1:  # if the robot is not facing the goal
            angular_vel = angle_error
            linear_vel = 0.0
        else:  # if the robot is facing the goal
            angular_vel = 0.0
            linear_vel = 0.5 * goal_dist

        # limit linear and angular velocities
        if linear_vel > 0.5:
            linear_vel = 0.5
        elif linear_vel < -0.5:
            linear_vel = -0.5
        if angular_vel > 1.0:
            angular_vel = 1.0
        elif angular_vel < -1.0:
            angular_vel = -1.0

        return [linear_vel, angular_vel]



    def shutdown(self):
        # stop turtlebot
        rospy.loginfo("Stop TurtleBot")
        # a default Twist has linear.x of 0 and angular.z of 0.  So it'll stop TurtleBot
        self.cmd_vel.publish(Twist())
        # sleep just makes sure TurtleBot receives the stop command prior to shutting down the script
        rospy.sleep(1)


    def callback(self,msg):
        # self coordinate
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        (_, _, yaw) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x,
                                                                msg.pose.pose.orientation.y,
                                                                msg.pose.pose.orientation.z,
                                                                msg.pose.pose.orientation.w])
        self.x = x 
        self.y = y 
        self.theta = yaw

if __name__ == '__main__':
    Test1()
    quit()
        
