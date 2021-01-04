from setuptools import setup, find_packages
from sys import platform as _platform
from distutils.extension import Extension
from distutils.util import get_platform
from glob import glob

setup(name="locomotion_simulation",
      version="0.0.1",
      packages=find_packages(),
      install_requires=["gym", "pybullet", "numpy", "scipy", "attrs"])
