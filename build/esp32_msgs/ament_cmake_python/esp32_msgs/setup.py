from setuptools import find_packages
from setuptools import setup

setup(
    name='esp32_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('esp32_msgs', 'esp32_msgs.*')),
)
