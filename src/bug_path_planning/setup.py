import os
from setuptools import find_packages, setup
from glob import glob

package_name = "bug_path_planning"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.py")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sasm",
    maintainer_email="sasilva1998@gmail.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "boundary_following = bug_path_planning.boundary_following:main",
            "motion_to_goal = bug_path_planning.motion_to_goal:main",
            "bug1 = bug_path_planning.bug1:main",
            "bug2 = bug_path_planning.bug2:main",
            "bug_tangent = bug_path_planning.bug_tangent:main",
        ],
    },
)
