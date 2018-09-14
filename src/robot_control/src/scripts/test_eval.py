import cv2 as cv
import numpy as np
from ctypes import *
import numpy.ctypeslib as npct

hsvlib = npct.load_library("librgb2hsv.so", "/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts")
array_3d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=3, flags=('CONTIGUOUS','WRITEABLE'))
array_2d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=2, flags=('CONTIGUOUS','WRITEABLE'))
array_1d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=1, flags=('CONTIGUOUS','WRITEABLE'))
hsvlib.rgb2hsv.argtypes = [array_3d_uint8t, c_int, c_int]
hsvlib.init.argtypes = [array_3d_uint8t]
hsvlib.evaluate.argtypes = [array_3d_uint8t, array_2d_uint8t, c_int, c_int] #, array_3d_uint8t]
positive_img = cv.imread("/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/positive.png")
hsvlib.init(positive_img)
img = cv.imread("image3.jpg")
height, width, channels = img.shape
hsvlib.rgb2hsv(img, width, height)
evaluated_img = np.zeros((60,60), np.uint8)
#dbg_img = np.zeros((height,width,3), np.uint8)
hsvlib.evaluate(img, evaluated_img, width, height) #, dbg_img)
cv.imshow('positive', positive_img)
#cv.imshow('dbg', dbg_img)
cv.imshow('hsv', img)
cv.imshow('evaluated', evaluated_img)
cv.waitKey(0)
cv.destroyAllWindows()
cv.imwrite("weird.png", img)
cv.imwrite("evaluated_image.png", evaluated_img)
#cv.imwrite("dbg_image.png", dbg_img)

