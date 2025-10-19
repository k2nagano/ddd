from setuptools import find_packages, setup

package_name = 'joy_hotplug_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/joy_hotplug.launch.py']),
        ('share/' + package_name + '/config', ['config/mappings.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nagano',
    maintainer_email='k2nagano@silver.ocn.ne.jp',
    description='Auto-switch joy_linux device & mapping on USB hotplug',
    license='MIT',
    # tests_require=['pytest'],
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'joy_manager = joy_hotplug_manager.joy_manager:main',
            'joy_mapper  = joy_hotplug_manager.joy_mapper:main',
        ],
    },
)
