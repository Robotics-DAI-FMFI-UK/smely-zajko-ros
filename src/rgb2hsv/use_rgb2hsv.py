import cv2 as cv
import numpy as np
from ctypes import *
import numpy.ctypeslib as npct


def convert_image():
  img = cv.imread("image.jpg")
  hsvlib = npct.load_library("librgb2hsv.so", ".")
  array_3d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=3, flags=('CONTIGUOUS','WRITEABLE'))
  hsvlib.rgb2hsv.argtypes = [array_3d_uint8t, c_int, c_int]
  height, width, channels = img.shape
  hsvlib.rgb2hsv(img, width, height)
  cv.imshow('converted', img)
  cv.waitKey(0)
  cv.destroyAllWindows()

convert_image()
