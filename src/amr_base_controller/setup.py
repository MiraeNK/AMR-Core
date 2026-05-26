
import os

from glob import glob

from setuptools import setup



package_name = 'amr_base_controller'



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

    description='AMR Polebot Base Controller — kinematics node',

    license='Apache-2.0',

    tests_require=['pytest'],

    entry_points={

        'console_scripts': [

            'kinematics_node = amr_base_controller.kinematics_node:main',

        ],

    },

)

