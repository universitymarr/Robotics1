from setuptools import setup, find_packages

setup(
    name='robobox',
    version='1.0.0',
    description='Toolbox for robotics applications',
    author='Mowibox',
    packages=find_packages(),
    install_requires=[
        'numpy',
        'sympy',
        'matplotlib',
    ]
)