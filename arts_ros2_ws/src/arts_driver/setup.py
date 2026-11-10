from setuptools import setup
from glob import glob
import os

package_name = 'arts_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*') if os.path.isdir('config') else []),
        ('share/' + package_name + '/urdf', glob('urdf/*') if os.path.isdir('urdf') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARTS Maintainers',
    maintainer_email='arts@anonymous.invalid',
    description='ARTS bringup launch and per-component configs.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'haply_bridge = arts_driver.haply_bridge:main',
        'arm_status = arts_driver.arm_status:main',
        'tool_info = arts_driver.tool_info:main',
        'cart_tf = arts_driver.cart_tf:main',
    ]},
)
