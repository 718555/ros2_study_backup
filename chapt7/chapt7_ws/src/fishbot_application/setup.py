from setuptools import setup

package_name = 'fishbot_application'

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
    maintainer='root',
    maintainer_email='1662683302@qq.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'init_robot_pose=fishbot_application.init_robot_pose:main',
            'get_robot_pose=fishbot_application.get_robot_pose:main',
        ],
    },
)
