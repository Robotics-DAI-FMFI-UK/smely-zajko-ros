rgb2hsv is a disconnected example that demonstrates a quick C implementation
of a transformation from RGB to HSV with ranges [0-255,0-255,0-255]
that can be called from Python on OpenCV image.

analyse is used to analyse training dataset - by counting the pixels of specific H, S values in the whole dataset, the resulting images positive.png and negative.png show their distribution. 

the folder hsv/Ledince480hsv/  should contain the HSV images, hsv/tren480 should contain the training set labeling, the program will output the evaluated input images using counting in the training set
