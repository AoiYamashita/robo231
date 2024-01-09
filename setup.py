import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robo231'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yamashita',
    maintainer_email='yamashitaaoi1230@icloud.com',
    description='test of using usb camera',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'receiver = robo231.receiver:main',
        ],
    },
)
