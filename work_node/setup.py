from setuptools import find_packages, setup

package_name = 'work_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Elio-Brayan',
    maintainer_email='eliotrianar95@gmail.com',
    description='work nodes - Robots Autonomos',
    license='License: Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'n_exploration = work_node.n_exploration:main',
            'n_detection = work_node.n_detection:main',
            'n_localization = work_node.n_localization:main'
        ],
    },
)
