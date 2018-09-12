from ctypes import *

def compute():
  print "calling c function simple(123)"
  lib = cdll.LoadLibrary(r"libsimple.so")
  val = lib.simple(123);
  print "function returned: " + str(val)

compute()

