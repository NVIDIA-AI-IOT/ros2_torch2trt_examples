from setuptools import setup

package_name = 'ros2_monocular_depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,package_name + '/MiDaS'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ak-nv',
    maintainer_email='ameykulkarni@nvidia.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_estimation = ros2_monocular_depth.midas_live:main',
        ],
    },
)
