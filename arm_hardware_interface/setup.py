from setuptools import setup
from setuptools import find_packages

package_name = 'arm_hardware_interface'

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
    description='Hardware Interface package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_pos_control = arm_hardware_interface.joint_pos_control:main',
            'hardware_interface = arm_hardware_interface.hardware_interface:main',
        ],
    },
)
