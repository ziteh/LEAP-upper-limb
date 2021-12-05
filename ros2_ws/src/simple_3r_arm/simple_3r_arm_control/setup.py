from setuptools import setup
from glob import glob

package_name = 'simple_3r_arm_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ("share/" + package_name, glob("launch/*.launch.py")),
        ("share/" + package_name + "/configs", glob("configs/*.*")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ziteh',
    maintainer_email='honmonoh@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "simple_3r_arm_joint_controller = simple_3r_arm_control.simple_3r_arm_joint_controller:main",
            "ik = simple_3r_arm_control.simple_3r_arm_inverse_kinematics:main",
            "fk = simple_3r_arm_control.simple_3r_arm_forward_kinematics_server:main",
            "relative_controller = simple_3r_arm_control.simple_3r_arm_relative_controller_service:main",
        ],
    },
)
