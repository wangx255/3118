from setuptools import setup
import os
from glob import glob

package_name = 'survey_missions'

setup(
    name=package_name, 
    version='0.0.1',  
    packages=[package_name], 
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],  
    zip_safe=True,
    maintainer='wenzhihu',  
    maintainer_email='HuW6@cardiff.ac.uk', 
    description='Ros2 homework',  
    license='MIT',  
    tests_require=['pytest'], 
    entry_points={
        'console_scripts': [
            'mission_handler = survey_missions.mission_handler:main' 
        ],
    },
)
