from setuptools import find_packages, setup

package_name = 'robot_motion'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='jimmy.byeon@gmail.com',
    description='Shared supervised navigation and encoder-feedback rotation.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motion_executor = robot_motion.executor_node:main',
            'motion_diagnostics = robot_motion.diagnostics_node:main',
            'grasp_video_recorder = robot_motion.video_recorder:main',
        ],
    },
)
