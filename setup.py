from setuptools import setup

package_name = 'twist_mux_dynamic'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/twist_mux_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/twist_mux_dynamic.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='uk',
    maintainer_email='tkddnr1022@dankook.ac.kr',
    description='Dynamic twist mux with lock-based priority control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_mux_dynamic = twist_mux_dynamic.twist_mux_dynamic_node:main'
        ],
    },
)
