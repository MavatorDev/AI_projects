#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajParams, CubicTrajCoeffs
from ar_interface.srv import ComputeCubicTraj

#Subscribes to the points, calls the service, and publishes coefficients.
class CubicTrajPlanner(Node):
    def __init__(self):
        super().__init__('cubic_traj_planner')
        #Subscrite to get params
        self.subscription = self.create_subscription(
            CubicTrajParams,
            'cubic_traj_params',
            self.listener_callback,
            10)
        #Create publisher to send computed coeffs
        self.publisher = self.create_publisher(CubicTrajCoeffs, 'Cubic_traj_coeffs', 10)
        #Call compute Cubic Trajectory service
        self.client = self.create_client(ComputeCubicTraj, 'Compute_cubic_traj')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Waiting for ComputeCubicTraj service")

    def listener_callback(self, msg):
        #Get params and call the service to cmpute coeffs
        request = ComputeCubicTraj.Request()
        request.params = msg
        future = self.client.call_async(request)
        #Call service callback method to publish coeffs into Cubic_traj_coeffs
        future.add_done_callback(self.service_callback)
        

    def service_callback(self, future):
        #Assing the coeffs returned by the service and publish them
        try:
            response = future.result()
            coeffs = CubicTrajCoeffs()
            coeffs.a0 = response.coeffs.a0
            coeffs.a1 = response.coeffs.a1
            coeffs.a2 = response.coeffs.a2
            coeffs.a3 = response.coeffs.a3
            coeffs.t0 = response.coeffs.t0
            coeffs.tf = response.coeffs.tf
            self.publisher.publish(coeffs)
            self.get_logger().info(f"Published: a0={coeffs.a0}, a1={coeffs.a1}, a2={coeffs.a2}, a3={coeffs.a3}, t0={coeffs.t0}, tf={coeffs.tf}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

def main(args=None):
    #Initialize node and maintain alive
    rclpy.init(args=args)
    node = CubicTrajPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()