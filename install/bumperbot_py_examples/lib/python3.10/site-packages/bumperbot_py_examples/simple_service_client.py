import rclpy
from rclpy.node import Node
from bumperbot_msgs.srv import AddTwoInts
import sys


class SimpleServiceClient(Node):
    def __init__(self, a, b):
        super().__init__("simple_service_client")

        self.client_ = self.create_client(AddTwoInts, "add_two_ints")
        #VERIFYING that actually this server is running and is availble to receive new requests
        while not self.client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not avalible, waiting again...")

        #onece we out of loop ros2 is ready to receive and to process new request messages. 
        #Message type that we are going to use :  AddTwoInts.Request()
        self.req_ = AddTwoInts.Request()
        self.req_.a = a
        self.req_.b = b

        #pass to this function the request message 
        #call_async :=> sends the request message to the service server and returns result && not actuall result is a future result
        self.future_ =  self.client_.call_async(self.req_)
        #Add a callback function that we can execute as soon as the service finishes its execution => return reesponse message 
        self.future_.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info("Service Response %d" % future.result().sum)


def main():
    # initialize ros2
    rclpy.init()
    
    if len(sys.argv) != 3:
        print("Wrong number of arguments! Usage: simple_service_client A B")
        return -1
    simple_service_client = SimpleServiceClient(int(sys.argv[1]), int(sys.argv[2]))
    #to make publisher still running || send a messages
    rclpy.spin(simple_service_client)
    # To close node
    simple_service_client.destroy_node()
    rclpy.shutdown()
if __name__ == '__main__':
    main()

        
