import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCost

class GetCostClient(Node):
    def __init__(self):
        super().__init__('get_cost_client')
        self.client = self.create_client(GetCost, 'get_cost')
        
    def send_request(self):
        request = GetCost.Request()
        request.use_footprint = True
        request.x = 0.0
        request.y = 0.0
        request.theta = 0.0
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = GetCostClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Service call completed')
    else:
        
        client.get_logger().error('Service call failed')
        
    client.destroy_node()
    rclpy.shutdown()