from setuptools import setup

package_name = 'bt_image_capture'

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
    maintainer='jinwoo',
    maintainer_email='jinwoo@example.com',
    description='BT image capture service node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bt_take_picture = bt_image_capture.take_picture_node:main',
        ],
    },
)
