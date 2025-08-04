#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu


# This class is implemented in order to improve the estimation of the angular velocity 
class KalmanFilter(Node):
    def __init__(self):
        super().__init__("kalman_filter")

        # Two subscriber to two ros2 topics 
        # one will receive the message coming from the IMU and the other one will receive the messages coming from encoders 
        self.odom_sub_ = self.create_subscription(Odometry,"bumperbot_controller/odom_noisy", self.odomCallback, 10)
        self.imu_sub_ = self.create_subscription(Imu, "imu/out", self.imuCallback, 10)
        # create a Ros2 publisher object that will publish instead the filtered data, after execution of the kalman filter --> odom_kalman =>publish result of kalman filter
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom_kalman", 10)

        # variables to implement logic of kalman filter algorithm 

        # mean of the Gaussian distribution that is referred to the angular velocity of the robot and also its variance
        # since we do not know its state of the robot or angular velocity we will put a high uncertainty value  
        self.mean_ = 0.0
        self.variance_ = 1000.0
        # Since we are going to implement a mono dimensional kalman filter, we are only going to filter the component of the angular velocity around the Z axis 
        # when ever we receive a new IMU measurement, we will need to update it and we will store the last estimation of the angular velocity oof the IMU 
        self.imu_angular_z_ = 0.0
        # we are going to use in order to identify the reception of the first wheel odometry message and that we are going to set it to false as soon as we receive the first odometry message   
        self.is_first_odom_ = True
        # will store the value of the last estimation of the angular velocity coming from wheel encoders sensors
        self.last_angular_z_ = 0.0
        # we store the difference between the angular velocity of the robot at two consecutive moments in time and this will contain the robot motion 
        self.motion_ = 0.0
        # This will contain the result of the kalman filter => the filtered odometry message that will be publish in the topic 
        self.kalman_odom_ = Odometry()
        # And this two indicates respectively the variance of random processs and so of the Gaussian distribution of the robot motion and of the inertial sensor
        self.motion_variance_ = 4.0 #robot motion and numbers are estimate and their only purpose is to configure the kalman filter
        
        self.measurement_variance_ = 0.5 #variance of the intertial sensor and numbers are estimate and their only purpose is to configure the kalman filter

    def measurementUpdate(self):
        #updated  mean 
        self.mean_ = (self.measurement_variance_ * self.mean_ + self.variance_ * self.imu_angular_z_) / (self.variance_ + self.measurement_variance_)
        #updated variance 
        self.variance_ = (self.variance_ * self.measurement_variance_) / (self.variance_ + self.measurement_variance_)

    def statePrediction(self):
        #updated  mean 
        self.mean_ = self.mean_ + self.motion_
        #updated variance
        self.variance_ = self.variance_ + self.motion_variance_
 
    def imuCallback(self, imu):
        self.imu_angular_z_ = imu.angular_velocity.z 
    def odomCallback(self, odom):
        # output of kalman_odom_ is exactly to the noisy one And then we are going to updates its values acccording to the result of the kalman filter.
        self.kalman_odom_ = odom

        if self.is_first_odom_:
            self.mean_ = odom.twist.twist.angular.z
            self.last_angular_z_ = odom.twist.twist.angular.z

            self.is_first_odom_ = False
            return
        
        self.motion_ = odom.twist.twist.angular.z - self.last_angular_z_
        
        self.statePrediction()

        self.measurementUpdate()

        self.kalman_odom_.twist.twist.angular.z = self.mean_
        self.odom_pub_.publish(self.kalman_odom_)

def main():
    # initialize ros2
    rclpy.init()
    kalman_filter = KalmanFilter()
    #to make publisher still running || send a messages
    rclpy.spin(kalman_filter)
    # To close node
    kalman_filter.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()