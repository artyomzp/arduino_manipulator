import rclpy
import sys
from rclpy.node import Node
from arduinobot_msgs.srv import AddTwoInts

class SimpleServiceCLient(Node):
    def __init__(self, a, b):
        super().__init__('simple_service_client')

        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available,waiting...')

        self.req = AddTwoInts.Request()
        self.req.a = a
        self.req.b = b
        self.future = self.client.call_async(self.req)
        self.future.add_done_callback(self.responseCallback)

    def responseCallback(self, future):
        self.get_logger().info("Server Response %d" % future.result().sum)

def main():
    rclpy.init()

    if len(sys.argv) != 3:
        print('Wrong number of arguments')
        return -1
    
    simple_service_client = SimpleServiceCLient(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin(simple_service_client)
    simple_service_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

