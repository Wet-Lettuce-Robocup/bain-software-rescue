import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'rescue_state_bc'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bain',
    maintainer_email='bchangson@proton.me',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'rescue_node = rescue_state_bc.rescue_node:main',
            'front_vision_node = rescue_state_bc.front_vision_node:main',
            'down_vision_node = rescue_state_bc.down_vision_node:main',
        ],
    },
)
