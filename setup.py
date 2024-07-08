import os
from glob import glob
from setuptools import setup

package_name = 'mcp9808_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (
            os.path.join("share", package_name, "launch"),
            glob("launch/*launch.[pxy][yma]*"),
        ),
    ],
    install_requires=['setuptools', 'adafruit-circuitpython-mcp9808'],
    zip_safe=True,
    maintainer='William Dubois',
    maintainer_email='william.dubois@usherbrooke.ca',
    description='ROS 2 package for the mcp9808 adafruit temperature sensor',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "mcp9808_node = mcp9808_ros.mcp9808_node:main",
        ],
    },
)
