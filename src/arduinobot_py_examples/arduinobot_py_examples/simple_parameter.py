import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

class SimpleParameter(Node):
    def __init__(self):
        super().__init__('simple_parameter')
        self.declare_parameter('int_param', 10)
        self.declare_parameter('string_param', 'Simple string')

        self.add_on_set_parameters_callback(self.paramChangeCallback)

    def paramChangeCallback(self, params):
        result = SetParametersResult()

        for param in params:
            if param.name == 'int_param' and param.Type == Parameter.Type.INTEGER:
                self.get_logger().info('Param --int_param-- changed to %d' % param)
                result.successful = True
            if param.name == 'string_param' and param.Type == Parameter.Type.STRING:
                self.get_logger().info('Param --string_param-- changed to %s' % param)
                result.successful = True
        return result
    
def main():
    rclpy.init()
    simple_parameter = SimpleParameter()
    rclpy.spin(simple_parameter)
    simple_parameter.destroy_node()
    rclpy.shutdown()

if __name__ == 'main':
    main()