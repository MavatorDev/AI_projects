from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    #Nodes to be launched

    points_generator_node = Node(
        package='ar_test',
        executable='points_generator',
        name='points_generator',
        output='screen'
    )

    cubic_traj_planner_node = Node(
        package='ar_test',
        executable='cubic_traj_planner',
        name='cubic_traj_planner',
        output='screen'
    )

    compute_cubic_coeffs_node = Node(
        package='ar_test',
        executable='compute_cubic_coeffs',
        name='compute_cubic_coeffs',
        output='screen'
    )

    plot_cubic_traj_node = Node(
        package='ar_test',
        executable='plot_cubic_traj',
        name='plot_cubic_traj',
        output='screen'
    )

    # Add the nodes to the launch description
    return LaunchDescription([
        points_generator_node,
        cubic_traj_planner_node,
        compute_cubic_coeffs_node,
        plot_cubic_traj_node
    ])