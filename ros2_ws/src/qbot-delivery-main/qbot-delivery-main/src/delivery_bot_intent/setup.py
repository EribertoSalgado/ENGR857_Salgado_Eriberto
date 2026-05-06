from setuptools import find_packages, setup

package_name = 'delivery_bot_intent'

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
    description='Intent communication node - translates state to color and display',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intent_communication = delivery_bot_intent.intent_communication_node:main',
        ],
    },
)
