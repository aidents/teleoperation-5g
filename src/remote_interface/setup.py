from setuptools import setup
import os
from glob import glob

package_name = 'remote_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'web'), glob('web/**/*', recursive=True)),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_email@ejemplo.com',
    description='Interfaz web remota para control de dron',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = remote_interface.web_server:main',
            'control_bridge = remote_interface.control_bridge:main',
            'ros2_web_bridge = remote_interface.ros2_web_bridge:main',
        ],
    },
) 