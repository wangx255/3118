from setuptools import find_packages, setup
import os
from glob import glob

package_name = "m2wr_description"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.gazebo")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*.world")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sasm",
    maintainer_email="sasilva1998@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tf_broadcaster = m2wr_description.tf_broadcaster:main",
            "groundtruth_broadcaster = m2wr_description.groundtruth_broadcaster:main",
        ],
    },
)
