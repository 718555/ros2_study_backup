from setuptools import setup

package_name = 'demo_python_tf'

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
            'static_tf_broadcaster=demo_python_tf.static_tf_broadcaster:main',
            'dynamic_tf_broadcaster=demo_python_tf.dynamic_tf_broadcaster:main',
            'tf_listener=demo_python_tf.tf_listener:main',
        ],
    },
)
