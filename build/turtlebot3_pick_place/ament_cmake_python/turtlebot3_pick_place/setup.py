from setuptools import find_packages
from setuptools import setup

setup(
    name='turtlebot3_pick_place',
    version='0.0.1',
    packages=find_packages(
        include=('turtlebot3_pick_place', 'turtlebot3_pick_place.*')),
)
