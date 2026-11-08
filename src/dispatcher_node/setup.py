from setuptools import find_packages, setup

package_name = "dispatcher_node"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools", 'networkx'],
    zip_safe=True,
    maintainer="Abdullah Shaikh",
    maintainer_email="abdullahshheikh2@gmail.com",
    description=(
        "Dispatcher node for TEMPO: sequences assigned subtasks "
        "wave-by-wave from the dependency DAG."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dispatcher_node = dispatcher_node.dispatcher_node:main",
        ],
    },
)