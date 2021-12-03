from setuptools import setup

package_name = 'data_package'

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
    maintainer='irs-lab-vm',
    maintainer_email='honmonoh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "sending_test = data_package.sending_test:main",
            "sending_test_bytemultiarray = data_package.sending_test_bytemultiarray:main",
            "motor_position_control_sender = data_package.motor_position_control_sender:main",
        ],
    },
)
