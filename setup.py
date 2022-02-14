from setuptools import setup

package_name = 'point_cloud_gen'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='anthony',
    maintainer_email='anthonybrown0528@protonmail.com',
    description='Contains ROS2 node that produces point cloud based on depth image and horizontal and vertical camera field of view',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gen_point_cloud = point_cloud_gen.gen_point_cloud:main'
        ],
    },
)
