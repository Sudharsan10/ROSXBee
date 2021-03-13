import os
from glob import glob
from setuptools import setup

package_name = 'rosxbeepy'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py'))  # include launch directory
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sudharsan',
    maintainer_email='sudharsansci@gmail.com',
    description='A ROS2 wrapper for xbee devices using digi-xbee python API',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'XBeeInterface = rosxbeepy.XBeeInterface:main'
        ],
    },
)
