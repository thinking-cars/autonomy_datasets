import os
from glob import glob
from setuptools import setup

package_name = 'autonomy_datasets'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[('share/ament_index/resource_index/packages',
                 ['resource/' + package_name]),
                (os.path.join('share', package_name), ['package.xml']),
                (os.path.join('share', package_name,
                              'launch'), glob('launch/*launch.[pxy][yma]*')),
                (os.path.join('share', package_name,
                              'config'), glob('config/*'))],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Raphael van Kempen',
    maintainer_email='vankempen@thinking-cars.de',
    description='Integrates automated driving datasets into the ROS 2 ecosystem',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts':
        ['autonomy_datasets = autonomy_datasets.autonomy_datasets:main'],
    },
)
