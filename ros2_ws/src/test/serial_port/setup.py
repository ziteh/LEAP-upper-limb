from setuptools import setup

package_name = 'serial_port'

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
            "basic_serial_port = serial_port.basic_serial_port:main",
            "asyn_serial_port = serial_port.asyn_serial_port:main",
            "thread_serial_port = serial_port.thread_serial_port:main",
            "read_write_from_topic = serial_port.read_write_from_topic:main",
        ],
    },
)
