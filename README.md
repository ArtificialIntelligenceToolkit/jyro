# Jyro: Jupyter & Robots

[![CircleCI](https://circleci.com/gh/Calysto/jyro/tree/master.svg?style=svg)](https://circleci.com/gh/Calysto/jyro/tree/master) [![codecov](https://codecov.io/gh/Calysto/jyro/branch/master/graph/badge.svg)](https://codecov.io/gh/Calysto/jyro) [![Documentation Status](https://readthedocs.org/projects/jyro/badge/?version=latest)](http://jyro.readthedocs.io/en/latest/?badge=latest) [![PyPI version](https://badge.fury.io/py/jyro.svg)](https://badge.fury.io/py/jyro)


Jyro contains a simulator (jyro.simulator) and controller for real robots (jyro.myro).

## Installation

```
pip3 install jyro -U
```

On MacOS, you may also need:

```bash
brew install cairo
```

## Documentation

See https://jyro.readthedocs.io

## jyro.myro

- Pictures now have OpenCV representation
```
  from myro import *
  init('/dev/tty.Fluke2-1234-Fluke2')
  p = takePicture()
  cv2.imshow('image', p.pixels)
```
- Graphics needs to be moved away from Tk

## jyro.myro Ancestors
### pyro
### IPRE myro
### IPRE's Calico Myro
