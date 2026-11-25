from setuptools import setup

package_name = 'tetra_odom_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='johny',
    maintainer_email='johny@hyulimatech.co.kr',
    description='Wheel odometry scale calibration driven from the TETRA web admin page',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrator = tetra_odom_calibration.calibrator_node:main',
        ],
    },
)
