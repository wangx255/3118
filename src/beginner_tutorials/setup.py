import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'beginner_tutorials'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='miku',
    maintainer_email='miku@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "velocity_pub_py = beginner_tutorials.velocity_pub:main",
        "velocity_sub_py = beginner_tutorials.velocity_sub:main",
        ],
    },
)
