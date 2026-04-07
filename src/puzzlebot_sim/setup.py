import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'puzzlebot_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instalar archivos de Launch
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Instalar archivos URDF
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # Instalar archivos de Mallas (STL)
        (os.path.join('share', package_name, 'meshes'), [
		'meshes/Puzzlebot_Jetson_Lidar_Edition_Base.stl',
        	'meshes/Puzzlebot_Wheel.stl',
            	'meshes/Puzzlebot_Caster_Wheel.stl'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tu_nombre',
    maintainer_email='tu@correo.com',
    description='Simulacion del Puzzlebot para el Mini Challenge 1',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
		'kinematics = puzzlebot_sim.kinematics:main',
        ],
    },
)
