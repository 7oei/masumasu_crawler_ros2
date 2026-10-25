from glob import glob
from setuptools import find_packages, setup

setup(
    name='masumasu_crawler', version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/masumasu_crawler']),
        ('share/masumasu_crawler', ['package.xml']),
        *[('share/masumasu_crawler/' + d, glob(d + '/' + pattern))
          for d, pattern in [('config', '*.yaml'), ('launch', '*.launch.py'),
                             ('udev', '*.rules*'), ('examples', '*.py')]],
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='7oei', maintainer_email='7oei@users.noreply.github.com',
    description='Standalone Masumasu crawler teleoperation and command interface',
    license='MIT',
    entry_points={'console_scripts': [
        'driver = masumasu_crawler.armadillo_driver:main',
        'teleop = masumasu_crawler.teleop:main',
        'odometry = masumasu_crawler.odometry:main',
    ]},
)
