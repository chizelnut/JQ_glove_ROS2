from setuptools import setup
from glob import glob

package_name = 'juqiao_glove'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/meshes', glob('meshes/*.stl')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='chizelnut',
    maintainer_email='noreply@example.com',
    description='ROS 2 driver and RViz visualization for the Juqiao Industrial fabric tactile glove.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'glove_node = juqiao_glove.glove_node:main',
            'viz_node = juqiao_glove.viz_node:main',
        ],
    },
)
