from setuptools import setup

package_name = 'publisher_s2c'

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
    maintainer='jaxa',
    maintainer_email='jaxa@jaxa.jp',
    description='[RACS2 Example] Publisher S2C',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'publisher_s2c_node = publisher_s2c.publisher_s2c_node:main'
        ],
    },
)
