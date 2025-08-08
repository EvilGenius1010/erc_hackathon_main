from setuptools import setup
import os

package_name = 'run'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), ['launch/warehouselaunch.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='Your package description',
    license='License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # your executables here
        ],
    },
)
