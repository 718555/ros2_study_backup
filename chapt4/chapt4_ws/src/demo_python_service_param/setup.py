from setuptools import setup

package_name = 'demo_python_service_param'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name+"/resource", ['resource/default.jpg','resource/test1.jpg']), # 左边为目标地址，右边为default.jpg相对setup.py的路径
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
            'learn_face_detect=demo_python_service_param.learn_face_detect:main',
            # 可执行文件 = 包名.可执行文件名字:函数名字
            'face_detect_node=demo_python_service_param.face_detect_node:main',
            'face_detect_client_node=demo_python_service_param.face_detect_client_node:main',
        ],
    },
)
