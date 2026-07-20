# Copyright Institute for Automotive Engineering (ika), RWTH Aachen University
# SPDX-License-Identifier: Apache-2.0

import os
from glob import glob

from setuptools import setup

package_name = "ros2_graph_export"

setup(
    name=package_name,
    version="1.0.0",
    packages=[package_name, f"{package_name}.templates"],
    package_data={"ros2_graph_export": ["templates/*.j2"]},
    include_package_data=True,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.[pxy][yma]*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (os.path.join("share", package_name, "templates"), glob("ros2_graph_export/templates/*")),
    ],
    install_requires=["setuptools", "Jinja2"],
    zip_safe=True,
    maintainer="Lennart Reiher",
    maintainer_email="lennart.reiher@rwth-aachen.de",
    description="Exports ROS 2 node and topic graphs",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["ros2_graph_export = ros2_graph_export.ros2_graph_export:main"],
    },
)
