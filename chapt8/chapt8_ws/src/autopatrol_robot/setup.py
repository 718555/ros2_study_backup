from setuptools import setup
from glob import glob

package_name = 'autopatrol_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+'/config', ['config/patrol_config.yaml']), # 把config/patrol_config.yaml拷贝到install下share/' + package_name+'/config
        ('share/' + package_name+'/launch', glob('launch/*.launch.py')), # 遍历相对目录launch下的所有launch.py
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='1662683302@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'patrol_node=autopatrol_robot.patrol_node:main',
            'speaker=autopatrol_robot.speaker:main',
        ],
    },
)
