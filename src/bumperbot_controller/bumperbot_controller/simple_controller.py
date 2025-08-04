#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import TwistStamped, TransformStamped
from sensor_msgs.msg import JointState
import numpy as np
from rclpy.time import Time
from rclpy.constants import S_TO_NS
from nav_msgs.msg import Odometry
import math
from tf_transformations import quaternion_from_euler
#to publish a transform between fixed reference frame called odom and a moving one called base_footprint => this transform is dynamic 
from tf2_ros import TransformBroadcaster


class SimpleController(Node):
    def __init__(self):
        super().__init__("simple_controller")
        #intialize two parameters wheel radius and separation 
        self.declare_parameter("wheel_radius",0.033)
        self.declare_parameter("wheel_separation",0.17)

        #To read its run time value the one that assign at the top of the nnode 
        self.wheel_radius_ = self.get_parameter("wheel_radius").get_parameter_value().double_value
        self.wheel_separation_ = self.get_parameter("wheel_separation").get_parameter_value().double_value

        #To print two parameter in Terminal 
        self.get_logger().info("Using wheel_radius %f " % self.wheel_radius_)
        self.get_logger().info("Using wheel_separation %f " % self.wheel_separation_)

        #intialize previous pose of the robot wheel 
        self.left_wheel_prev_pos_ = 0.0
        self.right_wheel_prev_pos_ = 0.0 
        #intialize the previouse time
        self.prev_time_ = self.get_clock().now()
        # Current value of the x,y coordinates and angle theta of the robot ... will update any time receive new joint state message 
        self.x_ = 0.0
        self.y_ = 0.0
        self.theta_ = 0.0  

        #create a publisher object To publish velocity commands to the wheel of the robot 
        self.wheel_cmd_pub_ = self.create_publisher(Float64MultiArray,"simple_velocity_controller/commands", 10)
        #Create subscriber that recieve velocity command that are publish
        self.vel_sub_ = self.create_subscription(TwistStamped,"bumperbot_controller/cmd_vel",self.velCallback, 10)
        # This subscriber will listen to joint_states topic.... at this topic it is going to publish at any moment in time the position of the wheels 
        #for the simulated robot .. this will be done by Ros2 Gazebo plugin and also it will publish message in this topic with the current status of the simulated wheel in the real robot 
        self.joint_sub_ = self.create_subscription(JointState,"joint_states", self.jointCallback,10)
        self.odom_pub_ = self.create_publisher(Odometry, "bumperbot_controller/odom",10)

        #defeing matrix that we are going to use to convert the velocity from the robot reference fram to the rotation of the wheels 
        self.speed_conversion_ = np.array([[self.wheel_radius_/2, self.wheel_radius_/2], 
                                           [self.wheel_radius_/self.wheel_separation_, -self.wheel_radius_/self.wheel_separation_]])
        
        self.odom_msg_ = Odometry()
        self.odom_msg_.header.frame_id = "odom"
        self.odom_msg_.child_frame_id = "base_footprint"
        self.odom_msg_.pose.pose.orientation.x = 0.0
        self.odom_msg_.pose.pose.orientation.y = 0.0
        self.odom_msg_.pose.pose.orientation.z = 0.0
        self.odom_msg_.pose.pose.orientation.w = 1.0

        #publication of the transform between odom frame intial reference frame fixed one   and  base_footprint frame attached to robot and moves along with it  
        self.br_ = TransformBroadcaster(self)
        self.transform_stamped_ = TransformStamped()
        #initializing this message here that we are going to publish with in tf topic 
        self.transform_stamped_.header.frame_id = "odom"
        self.transform_stamped_.child_frame_id = "base_footprint" 
        
        self.get_logger().info("The conversion matrix is %s" % self.speed_conversion_)

    #define callback function 
    def velCallback(self, msg):
        # An Array that access linear velocity and angular velocity by geomatry_msgs/msg/Twist and because the publish message will be TwistStamped
        robot_speed = np.array([[msg.twist.linear.x],
                                [msg.twist.angular.z]])
        # To compute velocity of the two wheel 
        wheel_speed = np.matmul(np.linalg.inv(self.speed_conversion_), robot_speed)
        #actual message that can publish to the wheel of the robot 
        wheel_speed_msg = Float64MultiArray()
        # insert a vector which contain a velocity for  each wheel of the robot  
        wheel_speed_msg.data = [wheel_speed[1, 0], wheel_speed[0, 0]]
        #To publish message and request ros2 library to support and move wheel 
        self.wheel_cmd_pub_.publish(wheel_speed_msg)

    def jointCallback(self,msg):
        # Contain the variation  of the position of left wheel postion[1] => postion of left wheel at the current moment in time 
        dp_left = msg.position[1] - self.left_wheel_prev_pos_
        dp_right = msg.position[0] - self.right_wheel_prev_pos_
        # calculate time to preform such a rotation   header.stamp => access current time 
        dt = Time.from_msg(msg.header.stamp) - self.prev_time_
        # updating previous left and right wheel and time 
        self.left_wheel_prev_pos_ = msg.position[1] 
        self.right_wheel_prev_pos_ = msg.position[0] 
        self.prev_time_ = Time.from_msg(msg.header.stamp) 
        # calculating velocity
        fi_left = dp_left / (dt.nanoseconds / S_TO_NS)
        fi_right = dp_right / (dt.nanoseconds / S_TO_NS)
        # calculating linear and angular velocity 
        linear = (self.wheel_radius_ * fi_right + self.wheel_radius_ * fi_left) / 2
        angular = (self.wheel_radius_ * fi_right - self.wheel_radius_ * fi_left) / self.wheel_separation_

        # calculate postion increment 
        d_s = (self.wheel_radius_ * dp_right + self.wheel_radius_ * dp_left) / 2
        # calculate orientation increment 
        d_theta = (self.wheel_radius_ * dp_right - self.wheel_radius_ * dp_left) / self.wheel_separation_
        # increment postion of robot and because d_s is a B dimensional vector so it has two components we want to use them 
        #to increment separately  the x and y coordinates to do that we use trigonometry and so  we use sin and cos function 
        #in order to calculate the respective components

        # increament orientation component
        self.theta_ += d_theta
         # increament X component
        self.x_ += d_s * math.cos(self.theta_)
         # increament Y component
        self.y_ += d_s * math.sin(self.theta_)

        q = quaternion_from_euler(0, 0, self.theta_)
        self.odom_msg_.pose.pose.orientation.x = q[0]
        self.odom_msg_.pose.pose.orientation.y = q[1]
        self.odom_msg_.pose.pose.orientation.z = q[2]
        self.odom_msg_.pose.pose.orientation.w = q[3]
        self.odom_msg_.header.stamp = self.get_clock().now().to_msg()
        #update postion of the robot a long x and y axis
        self.odom_msg_.pose.pose.position.x = self.x_
        self.odom_msg_.pose.pose.position.y = self.y_
        # update linear and angular velocity 
        self.odom_msg_.twist.twist.linear.x = linear
        self.odom_msg_.twist.twist.angular.z = angular

        #assign value for rotation matrix and one of the translation vector between two frames
        self.transform_stamped_.transform.translation.x = self.x_
        self.transform_stamped_.transform.translation.y = self.y_
        self.transform_stamped_.transform.rotation.x = q[0]
        self.transform_stamped_.transform.rotation.y = q[1]
        self.transform_stamped_.transform.rotation.z = q[2]
        self.transform_stamped_.transform.rotation.w = q[3]
        #set a time stamped when this  message  was generated 
        self.transform_stamped_.header.stamp = self.get_clock().now().to_msg()

        # self.get_logger().info("linear : %f , angular : %f" % (linear, angular))
        # self.get_logger().info("x : %f , y : %f , theta : %f" % (self.x_, self.y_, self.theta_)) 

        # instead of printing a message inn terminal let's publish it with in bumperbot controller odom topic
        self.odom_pub_.publish(self.odom_msg_)
        #to publish transform stamped message 
        self.br_.sendTransform(self.transform_stamped_)


def main():
    rclpy.init()
    simple_controller = SimpleController()
    rclpy.spin(simple_controller)
    simple_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()