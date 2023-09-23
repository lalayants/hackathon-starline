from setuptools import setup

package_name = 'explorer_wanderer'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='leo',
    maintainer_email='egormaga04@mail.ru',
    description='description',
    license='-',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wanderer_server = explorer_wanderer.wanderer_server:main',
        ],
    },
)
