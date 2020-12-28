from setuptools import setup

package_name = 'ds4_driver'

setup(
    name=package_name,
    version='0.2.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Naoki Mizuno',
    maintainer_email='naoki.mizuno.256@gmail.com',
    description='ROS driver for the DualShock 4 controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ds4_demo_node = ds4_driver.demo:main',
            'ds4_driver = ds4_driver.ds4_driver_node.main'
        ],
    },
)
