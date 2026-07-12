from setuptools import find_packages
from setuptools import setup

setup(
    name='rviz_car_sim',
    version='0.0.0',
    packages=find_packages(
        include=('rviz_car_sim', 'rviz_car_sim.*')),
)
