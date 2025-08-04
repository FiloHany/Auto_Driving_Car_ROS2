import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts

class SimpleServiceServer(Node) :
    def __init__(self):
        super().__init__("simple_service_server")

        # Function requires as input the message interface to be used for communication with 
        #the service server message and specifies how to request and response are structured 
        #use request and response that are declared in this interface AddTwoInts 
        self.service_ = self.create_service(AddTwoInts, "add_two_ints", self.serviceCallback)

        self.get_logger().info("Service add_two_ints Ready")


    def serviceCallback(self, req, res):
        self.get_logger().info("New Request Received a: %d, b: %d" % (req.a, req.b))
        res.sum = req.a + req.b
        self.get_logger().info("Returning sum: %d" % res.sum)

        return res 

def main():
    # initialize ros2
    rclpy.init()
    simple_service_server = SimpleServiceServer()
    #to make publisher still running || send a messages
    rclpy.spin(simple_service_server)
    # To close node
    simple_service_server.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()