#import library which allow us to use all ros2 functionality
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

#create a new Node

class SimplePublisher(Node):
    #initalize Node 
    def __init__(self):
        super().__init__("simple_publisher")
        #indicate the message type to exchange & and this pub_ will publisher a String msgs and topic name and size of msg queue
        self.pub_ = self.create_publisher(String,"chatter",10)
        #count number of msgs
        self.counter_ = 0
        #frequency of the publish msg
        self.frequency_ = 1.0
        #informative msg in termianl
        self.get_logger().info("Publishing at %d Hz" % self.frequency_)
        #function that allow us to define ros2 timer
        self.timer_ = self.create_timer(self.frequency_,self.timerCallback)
    # function callback when timmer execute
    def  timerCallback(self):
        msg = String()
        msg.data = "Hello ROS 2 - counter: %d" % self.counter_

        self.pub_.publish(msg)
        self.counter_+=1

def main():
    # initialize ros2
    rclpy.init()
    simple_publisher = SimplePublisher()
    #to make publisher still running || send a messages
    rclpy.spin(simple_publisher)
    # To close node
    simple_publisher.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()
         