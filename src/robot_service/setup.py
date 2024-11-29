from setuptools import find_packages, setup

package_name = "robot_service"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sasm",
    maintainer_email="sasilva1998@gmail.com",
    description="Demo service package",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "add_two_numbers_service = robot_service.server:main",
            "add_two_numbers_client = robot_service.client:main",
        ],
    },
)
