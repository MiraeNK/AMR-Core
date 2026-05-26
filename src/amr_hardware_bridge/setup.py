
import os

from glob import glob

from setuptools import setup



package_name = 'amr_hardware_bridge'



setup(

    name=package_name,

    version='1.0.0',

    packages=[package_name],

    data_files=[

        ('share/ament_index/resource_index/packages',

            ['resource/' + package_name]),

        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'config'),

            glob('config/*.yaml')),

    ],

    install_requires=['setuptools'],

    zip_safe=True,

    maintainer='Hafizh Husaini',

    maintainer_email='miraenk7@gmail.com',

    description='AMR Polebot Hardware Abstraction Layer — PLC driver node',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [

            'plc_driver_node = amr_hardware_bridge.plc_driver_node:main',

        ],

    },

)

