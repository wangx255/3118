from setuptools import find_packages, setup
from glob import glob
import os

package_name = "grid_planners_demo"

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
        (
            os.path.join("share", package_name, "config"),
            glob(os.path.join("config", "*.yaml")),
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
            "navigator_astar = grid_planners_demo.navigator_astar:main",
            "navigator_dijkstra = grid_planners_demo.navigator_dijkstra:main",
            "navigator_dstar_lite = grid_planners_demo.navigator_dstar_lite:main",
        ],
    },
)
