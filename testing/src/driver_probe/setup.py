from setuptools import find_packages, setup

package_name = "driver_probe"

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
    description="Observe-only instrumentation helper that a SUT node uses to report to the STC driver",
    license="Apache-2.0",
)
