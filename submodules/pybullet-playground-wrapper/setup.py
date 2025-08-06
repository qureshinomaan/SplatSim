from setuptools import setup, find_packages

setup(
    name="pybullet_playground",
    version="0.1.0",
    packages=find_packages(include=["pybullet_playground", "pybullet_playground/*"]),
)