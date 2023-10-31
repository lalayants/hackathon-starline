from setuptools import setup
import os
from glob import glob

package_name = '449_ADventure'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob(os.path.join('launch', '*launch.[pxy][yma]*'))), 
        (os.path.join('share', package_name, 'src'),glob(os.path.join('src', '*.py'))), 
        (os.path.join('share', package_name, 'config'),['config/nav_params.yaml']), 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vel_copy = 449_ADventure.velocity_sub:main',
            'victim_to_hero = 449_ADventure.hac:main',
        ],
    },
)
