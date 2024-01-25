from setuptools import find_packages, setup

package_name = 'planning'

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
    maintainer='aoki',
    maintainer_email='aaokilourenco@ua.pt',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = planning.publisher_member_function:main',
            'listener = planning.subscriber_member_function:main',
        ],
    },
)
