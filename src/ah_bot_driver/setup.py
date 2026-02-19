from setuptools import find_packages, setup

package_name = 'ah_bot_driver'

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
    maintainer='rhd0714',
    maintainer_email='rhd0714@todo.todo',
    description='AH Bot Driver Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'driver_node = ah_bot_driver.driver_node:main',
        ],
    },
)
