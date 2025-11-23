from setuptools import find_packages, setup

package_name = 'indy_dcp_bridge'

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
    maintainer='stl',
    maintainer_email='stl@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        # 'indy_dcp_joint_state_node = indy_dcp_bridge.joint_state_node:main',
        'indy_dcp_tcp_dither = indy_dcp_bridge.tcp_dither_test:main',
        'indy_dcp_fjt_server_node = indy_dcp_bridge.fjt_server_node:main',
    ],
},

)
