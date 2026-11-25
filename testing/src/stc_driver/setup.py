from setuptools import find_packages, setup

package_name = "stc_driver"

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
    description="Test driver that replays .rtspec forbidden traces against a ROS 2 SUT on simulated time",
    license="Apache-2.0",
    entry_points={"console_scripts": ["stc_driver = stc_driver.cli:run_stc_driver_and_exit"]},
)
