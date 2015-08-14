# Jyro: Jupyter & Robots

- Pictures now have OpenCV representation
```
  from myro import *
  init('/dev/tty.Fluke2-1234-Fluke2')
  p = takePicture()
  cv2.imshow('image', p.pixels)
```
- Graphics needs to be moved away from Tk

## Ancestors
### pyro
### IPRE myro
### IPRE's Calico Myro
