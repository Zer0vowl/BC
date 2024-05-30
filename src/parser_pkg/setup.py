from setuptools import find_packages, setup

package_name = 'parser_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'matplotlib'],
    zip_safe=True,
    maintainer='flejv',
    maintainer_email='erkoj.jirkakutyka@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'parser = parser_pkg.parser:main'
        ],
    },
)
