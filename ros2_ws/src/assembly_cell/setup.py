from setuptools import find_packages, setup

package_name = 'assembly_cell'

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
    maintainer='rokey',
    maintainer_email='rokeydg1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'assembly_node = assembly_cell.assembly_node:main',
            'assembly_cell = assembly_cell.assembly_cell:main',
            'task = assembly_cell.task:main',
            'area_detection_node = assembly_cell.area_detection_node:main'
        ],
    },
)