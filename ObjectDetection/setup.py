from setuptools import find_packages, setup

package_name = 'ObjectDetection'

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
    maintainer='backpack',
    maintainer_email='backpack@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "ObjectDetectorNode == ObjectDetection.ObjectDetection.object_detector:main"
        ],
    },
)
