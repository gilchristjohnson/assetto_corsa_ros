from setuptools import find_packages, setup

package_name = 'assetto_corsa_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'evdev'],
    zip_safe=True,
    maintainer='gil',
    maintainer_email='gil@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'start = assetto_corsa_bridge.assetto_corsa_bridge:main',
            'setup_controller = assetto_corsa_bridge.scripts.setup_controller:main',
        ],
    },
)
