import rclpy
from rclpy.node import Node
from ar_interface.msg import CubicTrajCoeffs
from std_msgs.msg import Float64
import numpy as np
import time

#Subscribes to coefficients and plots trajectories.
class PlotCubicTraj(Node):
    def __init__(self):
        super().__init__('plot_cubic_traj')
        self.subscription = self.create_subscription(
            CubicTrajCoeffs,
            'Cubic_traj_coeffs',
            self.listener_callback,
            10)
        # Create a publisher for the position trajectory, velocity trajectory and acceleration trajectory tocpis
        # Publishes messages of type Float64
        self.pos_pub = self.create_publisher(Float64, 'position_trajectory', 10)
        self.vel_pub = self.create_publisher(Float64, 'velocity_trajectory', 10)
        self.acc_pub = self.create_publisher(Float64, 'acceleration_trajectory', 10)

    def listener_callback(self, msg):
        t0 = msg.t0
        tf = msg.tf
        a0 = msg.a0
        a1 = msg.a1
        a2 = msg.a2
        a3 = msg.a3

        # Generate time array
        rate=0.1
        n_points = int((tf-t0)/rate)*1
        t = np.linspace(t0, tf, n_points)
        # Compute position, velocity, and acceleration for each t
        pos = a0 + a1 * t + a2 * t**2 + a3 * t**3
        vel = a1 + 2 * a2 * t + 3 * a3 * t**2
        acc = 2 * a2 + 6 * a3 * t

        # Publish trajectories
        for i in range(len(t)):
            pos_msg = Float64()
            pos_msg.data = pos[i]
            self.pos_pub.publish(pos_msg)

            vel_msg = Float64()
            vel_msg.data = vel[i]
            self.vel_pub.publish(vel_msg)

            acc_msg = Float64()
            acc_msg.data = acc[i]
            self.acc_pub.publish(acc_msg)
            time.sleep(rate)
    
        self.get_logger().info("Trajectories Published successfully")
        

def main(args=None):
    #Initialize node and maintain alive
    rclpy.init(args=args)
    #Call the created node
    node = PlotCubicTraj()
    rclpy.spin(node)
    #Destroy node when stopped
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()