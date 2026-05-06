from setuptools import find_packages, setup

package_name = 'fake_qbot'

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
    maintainer='Kovidh',
    maintainer_email='kovidh@example.com',
    description='Offline simulator for the QBot delivery robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_qbot = fake_qbot.fake_qbot_node:main',
        ],
    },
)