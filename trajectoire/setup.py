from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'trajectoire'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Indexation ROS 2
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Fichier package.xml
        ('share/' + package_name, ['package.xml']),
        # Tous les fichiers de lancement .py dans launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='@2ineddine',
    maintainer_email='zbou6599@gmail.com',
    description='Package for controlling robot trajectory.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'challenge1 = trajectoire.challenge1:main',
            'challenge2 = trajectoire.challenge2:main',
            'challenge3 = trajectoire.challenge3:main',
            'challenge2V0 = trajectoire.challenge2V0:main',
            'balle = trajectoire.balle:main',
        ],
    },
)
