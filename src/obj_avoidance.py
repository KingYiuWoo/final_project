#!/usr/bin/env python3
import rospy  # Python library for ROS
from sensor_msgs.msg import LaserScan  # lidar data
from nav_msgs.msg import Odometry  # odom data
from geometry_msgs.msg import Twist

class runRobot():  # main class

    def __init__(self, ns):  # main function
        self.twist = Twist()
        self.pub = rospy.Publisher(f"/{ns}/cmd_vel", Twist, queue_size=10)
        self.sub = rospy.Subscriber(f"/{ns}/scan", LaserScan, self.lidar_callback)
        self.sub = rospy.Subscriber(f"/{ns}/odom", Odometry, self.odom_callback)
        self.ns = ns
        self.last_pos = [0, 0, 0] # x,y positions and a counter

    def lidar_callback(self, msg):  # function for obstacle avoidance
        # "Safe distance"
        self.distance = 1.0
        # Check Obstacle at the front
        if msg.ranges[0] > self.distance and msg.ranges[15] > self.distance and msg.ranges[345] > self.distance:
            self.twist.linear.x = 1.0 # linear velocity
            self.twist.angular.z = 0.0
        else:
            rospy.loginfo(f"{self.ns}: Obstacle Detected")
            self.twist.linear.x = 0.0 # stop
            # Check Obstacle on the left
            if msg.ranges[15] <= self.distance:
                # rotate clock wise (i.e. turing right)
                if msg.ranges[45] <= self.distance:
                    # Turn further right
                    self.twist.angular.z = -0.5
                else:
                    self.twist.angular.z = -0.25
            # Check Obstacle on the right
            elif msg.ranges[345] <= self.distance:
                # rotate counter-clock wise (i.e. turing left)
                if msg.ranges[315] <= self.distance:
                    # Turning further left
                    self.twist.angular.z = 0.5
                else:
                    self.twist.angular.z = 0.25
            else:
                self.twist.angular.z = 0.25  # Turning left or right, doesn't matter
        self.pub.publish(self.twist)  # publish the move object

    def odom_callback(self, msg):
        # If the obstacle avoidance logic is calibrated properly (i.e. safe distance & velocities),
        #   ideally the robot won't get stuck
        # In case it gets stuck for more than a certain period, tell the robot to go backwards
        COUNTER_LIMIT = 50
        if self.last_pos[0] == round(msg.pose.pose.position.x, 3) and self.last_pos[1] == round(msg.pose.pose.position.y, 3):
            if self.last_pos[2] >= COUNTER_LIMIT:
                rospy.loginfo(f"{self.ns}: Stuck. Going backwards")
                self.twist.linear.x = -3.0
                self.twist.angular.z = 0.0
                self.pub.publish(self.twist)
            else:
                self.last_pos[2] = self.last_pos[2] + 1
        else:
            self.last_pos[0] = round(msg.pose.pose.position.x, 3)
            self.last_pos[1] = round(msg.pose.pose.position.y, 3)
            self.last_pos[2] = 0

if __name__ == '__main__':
    rospy.init_node('obstacle_avoidance_node')
    runRobot("tb3_0")  # run robot 0
    runRobot("tb3_1")  # run robot 1
    runRobot("tb3_2")  # run robot 2
    rospy.spin()