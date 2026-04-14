from setuptools import find_packages, setup

package_name = 'rover_base'

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
        'opencv-python',
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
            'camera_stream_node = rover_base.camera_stream_node:main',
            'self_test_node = rover_base.self_test_node:main',
        ],
    },
)
