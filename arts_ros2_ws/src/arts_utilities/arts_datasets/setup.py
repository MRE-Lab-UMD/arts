from setuptools import setup

package_name = 'arts_datasets'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARTS Maintainers',
    maintainer_email='arts@anonymous.invalid',
    description='MCAP to LeRobot dataset export.',
    license='Apache-2.0',
    entry_points={'console_scripts': []},
)
