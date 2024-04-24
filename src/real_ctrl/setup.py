from setuptools import find_packages, setup

package_name = 'real_ctrl'

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
    maintainer='jiahao',
    maintainer_email='54584749+LOOP115@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_trajectory_v1 = real_ctrl.follow_trajectory_v1:main',
            'follow_trajectory_v2 = real_ctrl.follow_trajectory_v2:main',
            'follow_trajectory_v3 = real_ctrl.follow_trajectory_v3:main',
            'move_to_pose = real_ctrl.move_to_pose:main',
            'follow_unity_target = real_ctrl.follow_unity_target:main',
            'move_to_start = real_ctrl.move_to_start:main',
            'trajectory_to_start = real_ctrl.trajectory_to_start:main',
        ],
    },
)
