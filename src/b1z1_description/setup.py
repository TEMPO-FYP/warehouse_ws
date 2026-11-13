import os
from setuptools import find_packages, setup

package_name = 'b1z1_description'


def generate_data_files(pkg_name, source_dir):
    """Recursively collects all files in source_dir for installation"""
    data_files = []
    for root, dirs, files in os.walk(source_dir):
        if files:
            install_dir = os.path.join('share', pkg_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_list))

    return data_files

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
]

data_files.extend(generate_data_files(package_name, 'xacro'))
data_files.extend(generate_data_files(package_name, 'meshes'))
data_files.extend(generate_data_files(package_name, 'launch'))
data_files.extend(generate_data_files(package_name, 'rviz'))


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Taha',
    maintainer_email='tz09220@st.habib.edu.pk',
    description='Robot description package for Unitree B1 quadruped with Z1 manipulator',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
