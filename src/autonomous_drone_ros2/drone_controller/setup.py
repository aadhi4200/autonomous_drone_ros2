from setuptools import setup
package_name = 'drone_controller'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={'console_scripts': ['waypoint_navigator = drone_controller.waypoint_navigator:main','aruco_landing_node = drone_controller.aruco_landing_node:main']},
)
