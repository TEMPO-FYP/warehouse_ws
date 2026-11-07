from setuptools import find_packages, setup

package_name = 'instruction_module'

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
    maintainer='Taha Zahid',
    maintainer_email='tz09220@st.habib.edu.pk',
    description='TEMPO Mission Instruction Node',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'instruction_node = instruction_module.instruction_node:main',
        ],
    },
)
