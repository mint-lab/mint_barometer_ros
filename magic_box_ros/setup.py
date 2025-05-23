import os
from glob import glob
from setuptools import setup

package_name = "magic_box_ros"
SHARE_DIR = os.path.join("share", package_name)

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join(SHARE_DIR, "launch"),
            glob(os.path.join("launch", "*.launch.py")),
        ),
        (os.path.join(SHARE_DIR, "config"), glob(os.path.join("config", "*"))),
        (os.path.join(SHARE_DIR, "rviz"), glob(os.path.join("rviz", "*.rviz"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mint",
    maintainer_email="hdwook3918@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ms5611_node = magic_box_ros.ms5611_node:main",
        ],
    },
)
