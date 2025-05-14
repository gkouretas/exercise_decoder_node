from setuptools import find_packages, setup

package_name = 'exercise_decoder_node'

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
    maintainer='georgekouretas',
    maintainer_email='thegreekgeorge07@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_speed_decoder = exercise_decoder_node.simple_speed_decoder:main',
            'force_mode_params_decoder = exercise_decoder_node.exercise_force_mode_params_decoder:main'
        ],
    },
)
