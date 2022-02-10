#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt

class TurtleBot:

    def __init__(self):

        rospy.init_node('turtlebot_squareclp', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.update_pose)
        self.pose = Pose()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.x = round(self.pose.x, 4)
        self.pose.y = round(self.pose.y, 4)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.x - self.pose.x), 2) + pow((goal_pose.y - self.pose.y), 2))

    def linear_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.y - self.pose.y, goal_pose.x - self.pose.x)

    def angular_vel(self, goal_pose, constant=9):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_pose) - self.pose.theta)

    def m2g(self):
        goal_pose = Pose()
        x = [5, 8, 8, 5, 5]
        y = [5, 5, 8, 8, 5]

        for i in range(5):
            goal_pose.x = x[i]
            goal_pose.y = y[i]
            d_tol = 0.01
            vel = Twist()

            while self.euclidean_distance(goal_pose) > d_tol:

                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control

                # Linear velocity in the x-axis.
                vel.linear.x = self.linear_vel(goal_pose)
                vel.linear.y = 0
                vel.linear.z = 0

                # Angular velocity in the z-axis.
                vel.angular.x = 0
                vel.angular.y = 0
                vel.angular.z = self.angular_vel(goal_pose)

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel)

                # Publish at the desired rate.
                self.rate.sleep()

            # Stopping our robot after the movement is over.
            vel.linear.x = 0
            vel.angular.z = 0
            self.velocity_publisher.publish(vel)

            # If we press control + C, the node will stop.
        rospy.spin()
        #for i in range(1):
            #d_tol = 0.01
            #vel = Twist()
            #goal_pose.x = 5 #x[i]
            #goal_pose.y = 5 #y[i]

            #while self.euclidean_distance(goal_pose) > d_tol:
                #vel.linear.x = self.linear_vel(goal_pose)
                #vel.linear.y = 0
                #vel.linear.z = 0
                #vel.angular.x = 0
                #vel.angular.x = 0
                #vel.angular.x = self.angular_vel(goal_pose)

                #self.velocity_publisher.publish(vel)
                #self.rate.sleep()

            #vel.linear.x = 0
            #vel.angular.z = 0
            #self.velocity_publisher.publish(vel)
        #rospy.spin()



if __name__ == '__main__':
	try:
		x = TurtleBot()
		x.m2g()
	except rospy.ROSInterruptException:
		pass
