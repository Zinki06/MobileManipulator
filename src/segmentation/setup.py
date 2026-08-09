import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'segmentation'

setup(
    name=package_name,
    version='0.0.0',
    # 'segmentation' 폴더 내의 __init__.py와 소스 파일들을 찾습니다.
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 사용을 대비해 미리 경로를 잡아둡니다.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='yu06663@gmail.com',
    description='YOLO and SAM segmentation node for object and hand masking.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # '실행할_명령어 = 패키지명.파일명:함수명'
            'segmentation_node = segmentation.segmentation_node:main'
        ],
    },
)