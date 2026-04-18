from setuptools import find_packages, setup

package_name = 'terrain_risk_layer'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'terrain_risk_node = terrain_risk_layer.terrain_risk_node:main',
            'data_logger       = terrain_risk_layer.data_logger:main',
        ],
    },
)
