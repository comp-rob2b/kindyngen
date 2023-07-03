#!/usr/bin/env python

from distutils.core import setup

setup(name='kindynsyn',
      version='0.1',
      description='Synthesize kinematics and dynamics algorithms for robotic manipulators',
      url='https://github.com/comp-rob2b',
      author='Sven Schneider',
      author_email='sven.schneider@h-brs.de',
      license='MPL-2.0',
      install_requires=['rdflib', 'pyshacl', 'numpy'],
      packages=['kindynsyn'],
      package_data={'kindynsyn': ['models/sparql/*.rq']}
     )
