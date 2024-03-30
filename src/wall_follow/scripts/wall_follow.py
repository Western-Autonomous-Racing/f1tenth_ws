#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

# PID CONTROL PARAMS
kp = 0.8
kd = 0.06
ki = 0.00005
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

# WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.9 # meters
DESIRED_DISTANCE_LEFT = 0.8
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow(Node):
    """ Implement Wall Following on the car
    """
    def __init__(self):
        super().__init__("WallFollow_node")
        # Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/nav'

        self.lidar_sub = self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, drive_topic, 10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.
        # TODO: implement
        distance_index = int(math.radians(angle) / data.angle_increment)
        distance = data.ranges[distance_index]
        return distance

    def pid_control(self, error, data):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        # TODO: Use kp, ki & kd to implement a PID controller
        integral += error * 0.1  # As sleep(0.1) so 0.1sec between scans.
        derivative = (error - prev_error) / 0.1
        angle = kp * error + ki * integral + kd * derivative
        prev_error = error
        if 0 <= math.degrees(abs(angle)) <= 10:
            velocity = 1.5
        elif 10 < math.degrees(abs(angle)) <= 20:
            velocity = 1
        else:
            velocity = 0.5
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = -angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def followLeft(self, data, leftDist):
        # Follow left wall as per the algorithm 
        # TODO: implement
        a = self.getRange(data, 225)
        b = self.getRange(data, 270)
        alpha = math.atan((a * math.cos(45) - b) / (a * math.sin(45)))
        D_t = b * math.cos(alpha)
        D_tplus1 = D_t + math.sin(alpha) * 2 * CAR_LENGTH
        # print(D_t)
        return leftDist - D_tplus1

    def lidar_callback(self, data):
        """ Lidar scan data used to calculate error for pid implementation
        """
        error = self.followLeft(data, DESIRED_DISTANCE_LEFT)  # TODO: replace with error returned by followLeft
        # send error to pid_control
        self.pid_control(error, data)


def main(args=None):
    rclpy.init(args=args)
    wf = WallFollow()
    rclpy.spin(wf)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
