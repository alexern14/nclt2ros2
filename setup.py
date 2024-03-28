from setuptools import find_packages, setup

package_name = "nclt2ros2"

setup(
    name=package_name,
    version="1.0.0",
    # packages=[package_name],
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="josh",
    maintainer_email="alex.m.ernst14@gmail.com",
    description="This node converts a twist message to a joystick message.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "nclt2rosbag2 = nclt2ros2.nclt2rosbag2:main",
        ],
    },
)
