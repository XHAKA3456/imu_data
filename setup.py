from setuptools import  setup

package_name = 'imu_receiver'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uk',
    maintainer_email='tkddnr1022@dankook.ac.kr',
    description='A ROS 2 node for receiving IMU data via UDP and publishing quaternion values',
    license='License',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub_imu = imu_receiver.pub_imu:main',
        ],
    },
)
