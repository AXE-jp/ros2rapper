from setuptools import find_packages, setup

package_name = 'py_pubsub'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='GPL-3.0-or-later',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = py_pubsub.pub:main',
            'sub_0 = py_pubsub.sub_0:main',
            'sub_1 = py_pubsub.sub_1:main',
            'sub_2 = py_pubsub.sub_2:main',
            'sub_3 = py_pubsub.sub_3:main',
            'sub_all = py_pubsub.sub_all:main',
        ],
    },
)
