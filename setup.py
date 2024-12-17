from setuptools import setup
from glob import glob
import os

package_name = 'realistic_error_injection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/resources/sample_neural_net_dataset4', glob(f'{package_name}/resources/sample_neural_net_dataset4/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='juan95',
    maintainer_email='jbarrag3@jh.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = realistic_error_injection.example_topic_pub:main',
        ],
    },
)
