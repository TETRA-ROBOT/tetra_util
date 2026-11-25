from setuptools import setup

package_name = 'tetra_imu_gate'

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
    description='Holds back IMU messages from the EKF while the wheels report standstill',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_standstill_gate = tetra_imu_gate.gate_node:main',
        ],
    },
)
