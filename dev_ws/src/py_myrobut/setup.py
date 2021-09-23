from setuptools import setup

package_name = 'py_myrobut'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, 'TB3'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name, 
                'resource/0.png', 
                'resource/1.png',
                'resource/2.png',
                'resource/3.png',
                'resource/4.png',
                'resource/5.png',
                'resource/6.png',
                'resource/7.png',
                'resource/8.png',
                'resource/9.png',
                'resource/dot.png',
                'resource/minus.png',
                'resource/myrcdash.png',
                'resource/red25x25.png',
                'resource/white25x25.png',

                ]),

        ('share/' + package_name, ['package.xml']),
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
            'service = py_myrobut.service_member_function:main',
            'client = py_myrobut.client_member_function:main',
        ],
    },
)
