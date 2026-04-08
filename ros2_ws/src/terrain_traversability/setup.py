from setuptools import setup

package_name = 'terrain_traversability'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ramana',
    maintainer_email='ramana@terrain_bot',
    description='Terrain traversability assessment node',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'traversability_node = terrain_traversability.traversability_node:main',
        ],
    },
)
