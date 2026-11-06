import os
from setuptools import find_packages, setup

package_name = 'llm_planner'


def generate_data_files(pkg_name, source_dir):
    data_files = []
    for root, dirs, files in os.walk(source_dir):
        if files:
            install_dir = os.path.join('share', pkg_name, root)
            file_list = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_list))
    return data_files


package_share_files = ['package.xml', '.env.example']

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name, package_share_files),
]

# Install launch files and knowledge base assets into share/llm_planner
data_files.extend(generate_data_files(package_name, 'launch'))
data_files.extend(generate_data_files(package_name, 'knowledge_base'))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'networkx'],
    zip_safe=True,
    maintainer='Taha Zahid',
    maintainer_email='tz09220@st.habib.edu.pk',
    description='TEMPO LLM Planner and Taxonomy Classification ROS 2 Package',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'planner_node = llm_planner.planner_node:main',
        ],
    },
)
