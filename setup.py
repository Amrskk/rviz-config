from setuptools import setup

package_name = 'lane_extractor_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='#YOUR NAME', # <-----
    maintainer_email='#YOUR EMAIL', # <-----
    description='Lane extractor node for LiDAR-based polyline visualization',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_extractor = lane_extractor_pkg.lane_extractor:main',
        ],
    },
)
