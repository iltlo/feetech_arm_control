from setuptools import setup
from setuptools import find_packages

package_name = 'arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, package_name + '.*']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iltlo',
    maintainer_email='iltlo@connect.hku.hk',
    description='Arm control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_pos_control = arm_control.joint_pos_control:main',
            'hardware_interface = arm_control.hardware_interface:main',
        ],
    },
)
