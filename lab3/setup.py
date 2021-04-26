from setuptools import setup

package_name = 'lab3'

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
    maintainer='mateusz',
    maintainer_email='matipalczuk@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'NONKDL_DKIN = lab3.NONKDL_DKIN:main',
            'KDL_DKIN = lab3.KDL_DKIN:main'
        ],
    },
)
