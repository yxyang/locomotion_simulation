from setuptools import setup, find_packages

setup(name="locomotion_simulation",
      version="0.0.1",
      packages=find_packages(),
      install_requires=["gym", "pybullet", "numpy", "scipy", "attrs"])
