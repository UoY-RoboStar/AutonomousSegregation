from setuptools import find_packages, setup

package_name = "random_walk"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Kumar",
    maintainer_email="kumaracharjee@outlook.com",
    description="RandomWalk SUT (ROS 2) for the STC test driver",
    license="Apache-2.0",
    entry_points={"console_scripts": ["random_walk = random_walk.random_walk_node:run_random_walk"]},
)
