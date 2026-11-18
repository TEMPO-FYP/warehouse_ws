from setuptools import find_packages, setup

package_name = 'robot_controllers'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config',
         ['config/controller_config.yaml', 'config/mobile_robot_localization.yaml']),
        ('share/' + package_name + '/launch',
         ['launch/mobile_robot_bringup.launch.py',
          'launch/mobile_robot_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Abdullah Shaikh',
    maintainer_email='abdullahshheikh2@gmail.com',
    description='Robot controller nodes for TEMPO.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mobile_robot_controller = robot_controllers.mobile_robot_controller:main',
        ],
    },
)
