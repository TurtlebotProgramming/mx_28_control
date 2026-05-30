from glob import glob
import os

from setuptools import find_packages, setup


package_name = 'mx_28_control'


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='syu',
    maintainer_email='yeonu0070@naver.com',
    description='Simple MX-28 angle controller driven by vision detections.',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mx_28_control = mx_28_control.mx_28_control:main',
        ],
    },
)
