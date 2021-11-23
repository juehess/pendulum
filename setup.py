import setuptools
from setuptools import setup, find_packages

setup(
    name='pendulum',
    version='0.1',
    packages=['pendulum', 'pendulum.driver'],
    url='https://github.com/juehess/pendulum',
    license='',
    author='Juergen Hess',
    author_email='juergen@hesspost.de',
    description='Inverted Pendulum Control',
    python_requires=">=3"
)
