#!/usr/bin/env python3

import rclpy
import rclpy.impl
from rclpy.node import Node
import time
from sensor_msgs.msg import Imu

# This python script simply republish the IMU data from one topic to another and change the frame ID in the topic message 

# we make it global in order to use it in callback function 
imu_pub = None

# Callback function that we want to execute whenever we receive a new IMU messages
def imuCallback(imu):
    global imu_pub
    # To change the frame id of the IMU messages that we have just receive
    imu.header.frame_id = "base_footprint_ekf"
    imu_pub.publish(imu)
    

def main():
    global imu_pub
    rclpy.init()
    # create ros2 node a new instance of this class
    node = Node("imu_republisher_node")
    # delay to wait our node to start publish
    time.sleep(1)
    # Publisher that will publish an imu messages 
    imu_pub = node.create_publisher(Imu, "imu_ekf", 10)
    # create a subscriber that will receive an imu messages that are declared in sensor messages
    imu_sub = node.create_subscription(Imu, "imu/out", imuCallback, 10)
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
