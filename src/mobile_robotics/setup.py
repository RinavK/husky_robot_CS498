from setuptools import find_packages, setup

package_name = 'mobile_robotics'

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
    maintainer='rinav',
    maintainer_email='rinav@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'part1 = mobile_robotics.coding_ex1_part1:main',
            'part2 = mobile_robotics.coding_ex1_part2:main',
            'ex2 = mobile_robotics.coding_ex2:main',
        ],
    },
)
