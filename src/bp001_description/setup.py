from setuptools import setup
from glob import glob

package_name = 'bp001_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.py')),
        ('share/' + package_name+'/urdf/', glob('urdf/*')),
        ('share/' + package_name+'/rviz/', glob('rviz/*')),
        ('share/' + package_name+'/meshes/collision/', glob('meshes/collision/*')),
        ('share/' + package_name+'/meshes/visual/', glob('meshes/visual/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umas',
    maintainer_email='1970313791@qq.com',
    description='Robot description package for the bp001 two-wheeled balance robot.',
    license='BSD',
    entry_points={
        'console_scripts': [
        ],
    },
)
