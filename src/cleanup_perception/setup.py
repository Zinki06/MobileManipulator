from glob import glob

from setuptools import find_packages, setup

package_name = 'cleanup_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jimmy.byeon@gmail.com',
    description='Request-driven YOLO and aligned-depth cleanup observations.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_perception_node = cleanup_perception.perception_node:main',
            'obstacle_depth_node = cleanup_perception.obstacle_depth_node:main',
            'grasp_trial = cleanup_perception.grasp_trial:main',
            'candidate_hover_trial = cleanup_perception.candidate_hover_trial:main',
        ],
    },
)
