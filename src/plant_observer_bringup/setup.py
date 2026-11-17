from glob import glob

from setuptools import setup

package_name = 'plant_observer_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/firmware', glob('firmware/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='eun',
    maintainer_email='djduddjdj@gmail.com',
    description='식물관찰 로봇 B안 실물 구동 노드와 통합 런치',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'base_driver = plant_observer_bringup.base_driver:main',
            'patrol_manager = plant_observer_bringup.patrol_manager:main',
            'nav2_session = plant_observer_bringup.nav2_session:main',
            'nav2_shutdown = plant_observer_bringup.nav2_session:shutdown_main',
            'change_compare = plant_observer_bringup.change_compare:main',
            'review_server = plant_observer_bringup.review_server:main',
            'capture_uploader = plant_observer_bringup.capture_uploader:main',
        ],
    },
)
