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
            'pub_0 = py_pubsub.pub_0:main',
            'pub_1 = py_pubsub.pub_1:main',
            'pub_2 = py_pubsub.pub_2:main',
            'pub_3 = py_pubsub.pub_3:main',
            'sub = py_pubsub.sub:main',
        ],
    },
)
