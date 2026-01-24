from setuptools import setup, find_packages

setup(
    name="movella_dot_py",
    version="0.1",
    packages=find_packages(),
    install_requires=[
        'bleak',
        'numpy',
    ],
)