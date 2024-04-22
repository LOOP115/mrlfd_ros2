from setuptools import find_packages, setup

package_name = 'ctrl_utils'

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
    maintainer='loop',
    maintainer_email='54584749+LOOP115@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "joints_publisher = ctrl_utils.joints_publisher:main",
            "move_to_start_server = ctrl_utils.move_to_start_server:main",
            "gripper_server = ctrl_utils.gripper_server:main",
        ],
    },
)
