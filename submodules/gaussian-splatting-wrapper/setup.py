from setuptools import setup, find_packages

setup(
    name="gaussian_splatting",
    version="0.1.0",
    packages=find_packages(include=["gaussian_splatting", "gaussian_splatting.*"]),
)