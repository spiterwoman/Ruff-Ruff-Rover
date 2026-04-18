from setuptools import find_packages, setup

package_name = 'rover_behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
            'numpy',
            'pyaudio',
            'pyusb',
        ],
        zip_safe=True,
    maintainer='mmamd',
    maintainer_email='mmamd@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'mic_whistle_node = rover_behavior.mic_whistle_ros2_node:main',
            'follow_whistler_node = rover_behavior.follow_whistler_node:main',
            'turn_to_whistle_test_node = rover_behavior.turn_to_whistle_test_node:main',
        ],
    },
)
