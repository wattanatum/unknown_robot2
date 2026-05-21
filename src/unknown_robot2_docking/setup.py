from setuptools import find_packages, setup

package_name = 'unknown_robot2_docking'

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
    maintainer='ghosts',
    maintainer_email='ghosts@example.com',
    description='Simple ArUco docking package for unknown_robot2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_simple_docking = unknown_robot2_docking.aruco_simple_docking:main',
        ],
    },
)
