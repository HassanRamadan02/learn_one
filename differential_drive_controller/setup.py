from setuptools import setup

package_name = 'differential_drive_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=['differential_drive_controller'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', ['scripts/waypoint_navigation.py']),  # Add your script here
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hassan',
    maintainer_email='hassan.ramadan1409@gmail.com',
    description='Differential Drive Controller Package with Waypoint Navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigation = differential_drive_controller.waypoint_navigation:main',  # This is the important part
        ],},
)
