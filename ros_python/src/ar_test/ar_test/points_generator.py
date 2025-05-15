import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajParams
import random

#Generates random values for initial and final position, velocity and time
class PointsGenerator(Node):
    def __init__(self):
        super().__init__('points_generator')
        #Create publisher to send messages of type cubic_taj_params
        self.publisher = self.create_publisher(CubicTrajParams, 'cubic_traj_params', 10)
        #Execute generate points every 10 seconds
        self.timer = self.create_timer(10.0, self.generate_points)

    def generate_points(self):
        msg = CubicTrajParams()
        #generate random values
        msg.p0 = random.uniform(-10, 10)
        msg.pf = random.uniform(-10, 10)
        msg.v0 = random.uniform(-10, 10)
        msg.vf = random.uniform(-10, 10)
        msg.t0 = 0.0
        msg.tf = msg.t0 + random.uniform(4,8)
        #Publish message
        self.publisher.publish(msg)
        self.get_logger().info(f"Values generated: p0={msg.p0}, pf={msg.pf}, v0={msg.v0}, vf={msg.vf}, t0={msg.t0}, tf={msg.tf}")

def main(args=None):
    #Initialize node and maintain alive
    rclpy.init(args=args)
    node = PointsGenerator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()