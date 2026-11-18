from setuptools import setup

package_name = 'tetra_relocalization'

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
    description='Re-localizes AMCL by matching the laser scan to the map when the pose is lost',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_relocalizer = tetra_relocalization.relocalizer_node:main',
        ],
    },
)
