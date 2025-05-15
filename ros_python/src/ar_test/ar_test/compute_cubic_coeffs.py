#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ar_interface.srv import ComputeCubicTraj

#Provides the service to compute coefficients.
class ComputeCubicCoeffs(Node):
    def __init__(self):
        super().__init__('compute_cubic_coeffs')
        self.srv = self.create_service(ComputeCubicTraj, 'Compute_cubic_traj', self.compute_coeffs_callback)

    def compute_coeffs_callback(self, request, response):
        #Get params from the request
        t0 = request.params.t0
        tf = request.params.tf
        dt = tf - t0
        p0 = request.params.p0
        pf = request.params.pf
        v0 = request.params.v0
        vf = request.params.vf
    
        # Compute coeffs using params
        response.coeffs.a0 = p0
        response.coeffs.a1 = v0
        response.coeffs.a2 = (3 * (pf - p0) - (2 * v0 + vf) * dt) / (dt ** 2)
        response.coeffs.a3 = (-2 * (pf - p0) + (v0 + vf) * dt) / (dt ** 3)
        response.coeffs.t0 = t0
        response.coeffs.tf = tf

        self.get_logger().info(f"Coeffs succesfully computed")
        
        return response

def main(args=None):
    #Initialize node and maintain alive
    rclpy.init(args=args)
    node = ComputeCubicCoeffs()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()