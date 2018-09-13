import cv2 as cv
import numpy as np
from ctypes import *
import numpy.ctypeslib as npct
import glob, os

def count_image(filename):
  filename2 = "../tren480/out_" + filename
  img = cv.imread(filename)
  img2 = cv.imread(filename2)
  height, width, channels = img.shape
  analib.ana(img, img2, width, height)

def transform_image(filename):
  img = cv.imread(filename)
  height, width, channels = img.shape
  analib.transform(img, width, height, positive);
  cv.imwrite("p_" + filename, img);
  img = cv.imread(filename)
  analib.transform(img, width, height, negative);
  cv.imwrite("n_" + filename, img);
  
analib = npct.load_library("libanalyse.so", ".")
array_3d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=3, flags=('CONTIGUOUS','WRITEABLE'))
array_2d_uint32t = npct.ndpointer(dtype=np.ubyte, ndim=3, flags=('CONTIGUOUS','WRITEABLE'))
analib.ana.argtypes = [array_3d_uint8t, array_3d_uint8t, c_int, c_int]
analib.dump_results.argtypes = [array_3d_uint8t, array_3d_uint8t]
analib.transform.argtypes = [array_3d_uint8t, c_int, c_int, array_3d_uint8t]
os.chdir("hsv/Lednice480hsv")
analib.init();
for file in glob.glob("*.png"):
  print(file)
  count_image(file)
positive = np.zeros((255,255,3), np.uint8)
negative = np.zeros((255,255,3), np.uint8)
print(analib.dump_results(positive, negative));

for file in glob.glob("*.png"):
  print(file)
  transform_image(file)

os.chdir("../..")
cv.imwrite("positive.png", positive)
cv.imwrite("negative.png", negative)
