from setuptools import find_packages, setup

package_name = 'ping_monitor'

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
    maintainer='hasegawa',
    maintainer_email='robohase01@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ping_monitor_node = ping_monitor.ping_monitor_node:main',
            'topic_ping_node = ping_monitor.topic_ping_node:main',
            'topic_echo_node = ping_monitor.topic_echo_node:main',
        ],
    },
)
