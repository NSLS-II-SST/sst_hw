from setuptools import setup, find_packages

with open('requirements.txt') as f:
    requirements = f.read().split()

setup(
    author="NIST Staff",
    author_email=None,
    description="SST hardware definitions",
    install_requires=requirements,
    name="sst_hw",
    packages=find_packages()
)
