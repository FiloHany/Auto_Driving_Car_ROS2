import rclpy
from rclpy.node import Node 
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def  __init__(self):
        super().__init__("simple_parameter")

        #To declare a parameter that are inherted from our node class
        self.declare_parameter("simple_int_param",28)
        self.declare_parameter("simple_string_param","FIlo")

        #allow us to define a callback function when ever one parameter or more are declare or change 
        self.add_on_set_parameters_callback(self.paramChangeCallback)

    # return a responds that have outcome of parameter change 
    def paramChangeCallback(self,params):
        result = SetParametersResult()

        #To loop over parameter and check if something happen 
        for param in params:
            #check first parameter of integer if something change and recived type of parameter still integer not any thing else 
            if param.name == "simple_int_param" and param.type_ == Parameter.Type.INTEGER:
                # To print an informative message in Terminal 
                self.get_logger().info("Param simple_int_param  changed New value is %d" % param.value)
                result.successful = True

            if param.name == "simple_string_param" and param.type_ == Parameter.Type.STRING:
                self.get_logger().info("Param simple_string_param  changed New value is %s" % param.value)
                result.successful = True

        return result

def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()