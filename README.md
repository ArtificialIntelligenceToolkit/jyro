# Jyro: Jupyter & Robots

Jyro contains a simulator (jyro.simulator) and controller for real robots (jyro.myro).

## Installation

```
pip3 install jyro -U
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
