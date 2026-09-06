from setuptools import find_packages, setup

package_name = 'cleanup_planner'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Schema-constrained Gemini planner with deterministic fallback.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gemini_planner_node = cleanup_planner.planner_node:main',
        ],
    },
)
