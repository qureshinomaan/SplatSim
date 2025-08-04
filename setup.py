from setuptools import setup, find_packages

setup(
    name="SplatSim",
    version="0.1.0",
    description="A simple simulation project using splats",
    author="Your Name",
    packages=find_packages(include=["splatsim", "splatsim.*"]),
    install_requires=[],  # Add your dependencies here if needed
    python_requires=">=3.7",
)