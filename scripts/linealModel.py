#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Pose2D

class rover_lineal:
    def __init__(self, l):
        # Declare the variables used
        self.__l = l
        self._last_time = 0.0

        # Rover states
        self.__pos = np.array([0.0, 0.0])
        self.__angle = 0.0

        # Declare the publish messages
        self.__states = Pose2D()

    # Wrap to pi function
    def __wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi

    # Get the time difference for dt
    def getDt(self):
        current_time = rospy.Time.now()
        self.__dt = (current_time - self._last_time).to_sec()
        self._last_time = current_time

    # Solve model
    def solveEquations(self, vl, vr):
        # Get rover velocities
        vel_lin = (vr + vl) / 2
        vel_ang = (vr - vl) / (2*self.__l)
        # Update rover angle
        self.__angle = self.__wrap_to_Pi(self.__angle + vel_ang*self.__dt)

        # Get rover position
        matrix = np.array([
            [1.0], # cos
            [0.0] # sin
        ])
        vector = np.array([vel_lin])
        result = np.dot(matrix, vector)

        # Update rover position
        self.__pos += result * self.__dt
    
    # Get the position of the rover
    def getStates(self):
        self.__states.theta = self.__angle
        self.__states.x, self.__states.y = self.__pos[0], self.__pos[1]
        return self.__states

if __name__ == "__main__":
    # Initialise and Setup node
    rospy.init_node('Lineal_Model_Rover')

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("~node_rate", 1000))

    # Setup de publishers
    pub = rospy.Publisher('/lineal_pose', Pose2D, queue_size=10)

    # Classes
    rover = rover_lineal(l=12.5)
    vel_left = 0.0
    vel_right = 5.0

    # Wait to the rqt_multiplot to be ready
    rospy.sleep(5)
    print("The Lineal Model for the Rover is Running")
    try:
        while not rospy.is_shutdown():
            if not rover._last_time:
                rover._last_time = rospy.Time.now()
            else:
                rover.getDt()
                rover.solveEquations(vel_left, vel_right)
                
                # Publish the position
                pub.publish(rover.getStates())
            rate.sleep()

    except rospy.ROSInterruptException:
        pass