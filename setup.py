from setuptools import find_packages, setup

package_name = 'visualize_ros2bag'

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
    maintainer='Yaeohn Kim',
    maintainer_email='BlackTea12@github.com',
    description='visualize_ros2bag package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracking = visualize_ros2bag.tracking:main',
        ],
    },
)
