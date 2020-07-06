from setuptools import setup

package_name = 'conditions'

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
    maintainer='craig',
    maintainer_email='craig.hickman@ukaea.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'condition_publisher = conditions.condition_publisher_cli:main',
            'message_equality_tester = conditions.message_equality_tester_cli:main',
            'multi_message_equality_tester = conditions.multi_message_equality_tester_cli:main'
        ],
    },
)
