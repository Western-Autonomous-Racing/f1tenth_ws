#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import math

import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped

# kp = 0.8
# kd = 0.06
# ki = 0.00005
# servo_offset = 0.0
# prev_error = 0.0 
# error = 0.0
# integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow(Node):
    """ 
    Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__('wall_follow_node')

        lidarscan_topic = '/scan'
        drive_topic = '/drive'

        # TODO: create subscribers and publishers
        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.scan_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

        # TODO: set PID gains
        self.kp = 0.8
        self.kd = 0.06
        self.ki = 0.00005

        # TODO: store history
        self.integral = 0.0
        self.prev_error = 0.0
        self.error = 0.0

        # TODO: store any necessary values you think you'll need

    def get_range(self, range_data, angle):
        """
        Simple helper to return the corresponding range measurement at a given angle. Make sure you take care of NaNs and infs.

        Args:
            range_data: single range array from the LiDAR
            angle: between angle_min and angle_max of the LiDAR

        Returns:
            range: range measurement in meters at the given angle

        """

        #TODO: implement
        distance_index = int(math.radians(angle)/range_data.angle_increment)
        distance = range_data.ranges[distance_index]
        return distance

    # def get_error(self, range_data, dist):
    #     """
    #     Calculates the error to the wall. Follow the wall to the left (going counter clockwise in the Levine loop). You potentially will need to use get_range()

    #     Args:
    #         range_data: single range array from the LiDAR
    #         dist: desired distance to the wall

    #     Returns:
    #         error: calculated error
    #     """

    #     #TODO:implement
    #     return 0.0

    def pid_control(self, error, velocity):
        """
        Based on the calculated error, publish vehicle control

        Args:
            error: calculated error
            velocity: desired velocity

        Returns:
            None
        """
        angle = 0.0
        # TODO: Use kp, ki & kd to implement a PID controller
        # drive_msg = AckermannDriveStamped()
        # TODO: fill in drive message and publish
        integral += error * 0.1 #As sleep(0.1) so 0.1sec between scans.
        derivative = (error - prev_error) / 0.1
        angle = self.kp*error + self.ki*integral + self.kd*derivative
        prev_error = error
        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 1.5
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 1
        else:
            velocity = 0.5
        drive_msg = AckermannDriveStamped()
        # drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def scan_callback(self, msg):
        """
        Callback function for LaserScan messages. Calculate the error and publish the drive message in this function.

        Args:
            msg: Incoming LaserScan message

        Returns:
            None
        """
        error = self.followLeft(msg, DESIRED_DISTANCE_LEFT)
        self.pid_control(error, msg)
        # error = 0.0 # TODO: replace with error calculated by get_error()
        # velocity = 0.0 # TODO: calculate desired car velocity based on error
        # self.pid_control(error, velocity) # TODO: actuate the car with PID
    def followLeft(self, data, leftDist):
        #Follow left wall as per the algorithm
        a = self.getRange(data,225)
        b = self.getRange(data,270)
        alpha = math.atan((a*math.cos(45) - b)/(a*math.sin(45)))
        D_t = b*math.cos(alpha)
        D_tplus1 = D_t + math.sin(alpha) *2*CAR_LENGTH
        #print(D_t)
        return leftDist - D_tplus1

def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    wall_follow_node = WallFollow()
    rclpy.spin(wall_follow_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()