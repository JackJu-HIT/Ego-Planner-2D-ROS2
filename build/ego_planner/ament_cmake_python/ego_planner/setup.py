from setuptools import find_packages
from setuptools import setup

setup(
    name='ego_planner',
    version='0.0.0',
    packages=find_packages(
        include=('ego_planner', 'ego_planner.*')),
)
