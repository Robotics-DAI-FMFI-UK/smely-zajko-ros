#!/usr/bin/env python

import cv2 as cv
import numpy as np

import resnet_model as resnet


# TEST SEGMENTACNEJ NEURONKY
model = resnet.resnet()
model.load_weights('resnet.hdf5')


#def callback(cv_image):
#    global counterr
#    print "a"
#    cv_image = cv.flip(cv_image, -1)
#    print "a"
#    #pub = rospy.Publisher('camera_prediction', Image, queue_size=1)
#    ###pub_triangle = rospy.Publisher('camera_triangles_prediction', Float64MultiArray, queue_size=1)
#    #pub_eval = rospy.Publisher('/sensors/camera/evaluated_image', UInt8MultiArray, queue_size=3)
#    try:
#        cv_image = cv.resize(cv_image, (640, 480))
#        print "a"
#        
#        cvi_height, cvi_width, cvi_channels = cv_image.shape
#        ###hsvlib.rgb2hsv(cv_image, cvi_width, cvi_height)
#        ###hsvlib.evaluate(cv_image, evaluated_img, cvi_width, cvi_height)
#        
#        #print("publishing evaluated image" + str(counterr))
#        counterr += 1
#
#        if NEURON_IN_USE:
#            ###X = prepare_image(cv_image, window=window, stride=stride)
#            ###X = (X - 87.062)
#            ###X = (X / 255.0)
#            cv_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
#            X = cv_image / 255
#            y_pred = model.predict(np.expand_dims(X, axis=0))
#	    y_pred[y_pred <= 0.5] = 0
#            y_pred[y_pred > 0.5] = 255
#
#            prediction_mask = y_pred[0].astype(np.uint8)
#
#            ###prediction_mask = ((y_pred[:, 0].reshape(96, 128) * 255)).astype('uint8')
#            #prediction_mask = (y_pred.reshape(96, 128) * 255).astype('uint8')
#    
#            # nove:
#            # prediction_mask =  (y_pred.reshape(96, 128) * 255).astype('uint8')
#    
#            # print(prediction_mask.max(), prediction_mask.min(), prediction_mask.mean())
#    	
#            cvi_height, cvi_width, cvi_channels = prediction_mask.shape
#            hsvlib.evaluate(prediction_mask, evaluated_img, cvi_width, cvi_height)
#            #TODO: send prediction_mask and evaluate_img over socket to zajko
#
#            #print("publishing evaluated image" + str(counterr))
#            counterr += 1
#            eval_msg = UInt8MultiArray()
#            #print("published evaluated image")


def main():
    cv_image = cv.imread('testing_image.png')
    rgb_image = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
    X = rgb_image / 255
    print("Starting prediction")
    y_pred = model.predict(np.expand_dims(X, axis=0))
    y_pred[y_pred <= 0.5] = 0
    y_pred[y_pred > 0.5] = 255

    cv.imshow('image', cv_image)
    cv.imshow('prediction', y_pred[0])
    cv.waitKey(0)

    #while running:
    #    l, m = recv_msg(sock)
    #    print l
    #    img = cv.imdecode(np.fromstring(m, np.uint8), cv.IMREAD_COLOR)
    #    callback(img)
    #    if action == 1:
    #        detectQR(img)

    #    #cv.imshow('camera', img)
    #    #cv.waitKey(1)

if __name__ == '__main__':
    main()


