from setuptools import setup

package_name = 'mujoco_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={package_name: package_name},
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/example.py']),
        ('share/' + package_name + '/models', ['models/scene.xml','models/ur5e.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Student Team',
    maintainer_email='you@example.com',
    description='UR5e MuJoCo ROS2 integration — Milestones 2 & 3',
    entry_points={
        'console_scripts': [
            'mujoco_interface = mujoco_ros2.mujoco_interface:main',
            'sliders_node = mujoco_ros2.sliders_node:main',
            'fk_node = mujoco_ros2.fk_node:main',
        ],
    },
)
