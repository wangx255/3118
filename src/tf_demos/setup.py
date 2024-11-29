from setuptools import find_packages, setup
import os
from glob import glob

package_name = "tf_demos"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share/", package_name), glob("launch/*.launch.py")),
        (os.path.join("share/", package_name), glob("rviz/*.rviz")),
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
            "quaternions_conversion = tf_demos.quaternions_conversion:main",
            "tf_structures = tf_demos.tf_structures:main",
            "tf_broadcaster = tf_demos.tf_broadcaster:main",
            "camera_link_broadcaster = tf_demos.camera_link_broadcaster:main",
            "static_tf_broadcaster = tf_demos.static_tf_broadcaster:main",
            "can_link_broadcaster = tf_demos.can_link_broadcaster:main",
            "transformation_matrix = tf_demos.transformation_matrix:main",
        ],
    },
)
