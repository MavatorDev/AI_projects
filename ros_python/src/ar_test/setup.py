from setuptools import find_packages, setup

package_name = 'ar_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['ar_test/launch/cubic_traj_gen.launch.py']), #Launch file
    ],
    #Dependencies
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'ar_interface'],
    zip_safe=True,
    #Developer Info Mavator from -> MAnuela VAlencia TORo
    maintainer='mavator',
    maintainer_email='mavator@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #All nodes to be executed and the function to call
            "points_generator = ar_test.points_generator:main", #Executable=pkg.filename:function
            "cubic_traj_planner = ar_test.cubic_traj_planner:main",
            "compute_cubic_coeffs = ar_test.compute_cubic_coeffs:main",
            "plot_cubic_traj = ar_test.plot_cubic_traj:main"
        ],
    },
)
