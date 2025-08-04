import rclpy
from rclpy.node import Node
#to publish a transform  betwwen two fixed reference frame in which the relative postion and orientation do not change
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster, TransformException
# To get Transform between the  frames that are available in the TF and in the TF static so, we will import listener and buffer  
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
#the message type that is accepted in the TF and the TF static topic called transform stamped aand it is defined in the geometry messages 
from geometry_msgs.msg import TransformStamped
from bumperbot_msgs.srv import GetTransform
#To multiply two quaternions and  so in order to multiply them => quaternion_multiply.  quaternion_inverse => to stop and move in opposite direction after a certain speed
from tf_transformations import quaternion_from_euler, quaternion_multiply, quaternion_inverse


class SimpleTfKinematics(Node):
    def __init__(self):
        super().__init__("simple_tf_kinematics")
        
        self.static_tf_broadcaster_ = StaticTransformBroadcaster(self)
        #To publish a transform message 
        self.dynamic_tf_broadcaster_ = TransformBroadcaster(self)
        
        # indicates the increment that we want to add to the transform at any iteration of the timer 
        self.x_increment_ = 0.05 
        # contain last known value of the transform along the x direction that well calculate next position  
        self.last_x_ = 0.0
        #This will count the number of rotation that the bumperbot base frame is doing with respect to the odom frame 
        self.rotations_counter_ = 0
        #contains the last known orientation of the moving frame 
        self.last_orientation_ = quaternion_from_euler(0, 0, 0)
        #increment in the orientation of the bumperbot base frame. 0.05 is yaw angle a long z axis and it increment of 0.05 radians when ever time expires   
        self.orientation_increment_ = quaternion_from_euler(0, 0, 0.05)

        self.tf_buffer_ = Buffer()
        self.tf_listener_ = TransformListener(self.tf_buffer_, self)

        self.static_transform_stamped_ = TransformStamped()
        #To publish a transform message
        self.dynamic_transform_stamped_ = TransformStamped()
        #add information about the time when this transform has been generated ... stampe => that represent the time stamp when this message was generated 
        self.static_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        #Define the names of the two frames that are connected by these static transform 
        self.static_transform_stamped_.header.frame_id = "bumperbot_base"
        self.static_transform_stamped_.child_frame_id = "bumperbot_top"
        #Define characteristics of this condition so we have to define the rotation matrix and translation vector that defines the connections 
        self.static_transform_stamped_.transform.translation.x = 0.0
        self.static_transform_stamped_.transform.translation.y = 0.0
        self.static_transform_stamped_.transform.translation.z = 0.3
        self.static_transform_stamped_.transform.rotation.x = 0.0
        self.static_transform_stamped_.transform.rotation.y = 0.0
        self.static_transform_stamped_.transform.rotation.z = 0.0
        self.static_transform_stamped_.transform.rotation.w = 1.0
        
        # self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        # self.dynamic_transform_stamped_.header.frame_id = "odom"
        # #Our Dynamic connection 
        # self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
         
        #To publish this transform 
        self.static_tf_broadcaster_.sendTransform(self.static_transform_stamped_)

        self.get_logger().info("Publishing static transform between %s and %s" % 
                               (self.static_transform_stamped_.header.frame_id, self.static_transform_stamped_.child_frame_id))
        # To publish the dynamic transform to simulate the relative movement between two frames we use ROs2 timer to execute a certain function 
        # every 0.1 will publish a new transformation of the message 
        #when timer expired we want to  execute the function timer callback
        self.timer_ = self.create_timer(0.1, self.timerCallback)
        #create a service 
        self.get_transform_srv_ = self.create_service(GetTransform, "get_transform", self.getTransformCallback)


    def timerCallback(self):
        self.dynamic_transform_stamped_.header.stamp = self.get_clock().now().to_msg()
        self.dynamic_transform_stamped_.header.frame_id = "odom"
        #Our Dynamic connection 
        self.dynamic_transform_stamped_.child_frame_id = "bumperbot_base"
        self.dynamic_transform_stamped_.transform.translation.x = self.last_x_ + self.x_increment_
        self.dynamic_transform_stamped_.transform.translation.y = 0.0
        self.dynamic_transform_stamped_.transform.translation.z = 0.0
        # calculating the new orientation that the bumperbot_base frame will have with the respect to the bottom frame.   
        q = quaternion_multiply(self.last_orientation_, self.orientation_increment_)
        #this indicates bumperbot_base and odom frame are always oriented in the same way 
        #so now we have set that the rotation matrix between the bumperbot base and other frame is given by is quaternion 
        #that we have obtained by multiplying the last orientation tha increment , certain increment in the angle so , in the orientation of the two frame  
        self.dynamic_transform_stamped_.transform.rotation.x = q[0]
        self.dynamic_transform_stamped_.transform.rotation.y = q[1]
        self.dynamic_transform_stamped_.transform.rotation.z = q[2]
        self.dynamic_transform_stamped_.transform.rotation.w = q[3]

        self.dynamic_tf_broadcaster_.sendTransform(self.dynamic_transform_stamped_)

        self.last_x_ = self.dynamic_transform_stamped_.transform.translation.x

        #update the new support variables that we have created .... updating the rotation counter , last orientation variable 
        self.rotations_counter_ +=1
        self.last_orientation_ = q
        #check wether rotation counter is bigger than 100 => certain threshold that we can set the robot will stop and move to the opposite direction   
        if self.rotations_counter_ >= 100:
            self.orientation_increment_ = quaternion_inverse(self.orientation_increment_)
            self.rotations_counter_ = 0

    def getTransformCallback(self, req, res):
        self.get_logger().info("Requested Transform between %s and %s" % (req.frame_id, req.child_frame_id))
        #Transform stamped variable that actually will contain the requested transform  between two selected frames 
        requested_transform = TransformStamped()
        #To obtain transform between two frames that is published in TF and TF static topic we can use the lookup transform function from the buffer library 
        #lookup_transform => allow us to use TF library in order to know the current transformation matrix between any two frames 
        try:
            requested_transform = self.tf_buffer_.lookup_transform(req.frame_id, req.child_frame_id, rclpy.time.Time())
        except TransformException as e :
            self.get_logger().info("An error occurred while transforming %s  and %s" % (req.frame_id, req.child_frame_id))
            res.success = False
            return res
        res.transform = requested_transform
        res.success = True
        return res         

def main():
    # initialize ros2
    rclpy.init()
    simple_tf_kinematics = SimpleTfKinematics()
    #to make publisher still running || send a messages
    rclpy.spin(simple_tf_kinematics)
    # To close node
    simple_tf_kinematics.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()