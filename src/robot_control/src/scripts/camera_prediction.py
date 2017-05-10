#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from skimage.util.shape import view_as_windows
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from smely_zajko_dataset import models

bridge = CvBridge()

window = (5, 5)
stride = 5

model = models.mlp(n_input=75, architecture=[(20, 'sigmoid'), (2, 'softmax')],
                   metrics=['accuracy'])
model.load_weights('/home/zajko/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/mlp_20_sigmoid_2_softmax.hdf5')

def prepare_image(image, window, stride):
    window_x, window_y = window
    Y = np.asarray(np.squeeze(view_as_windows(image,
                                              (window_x, window_y, 3),
                                              step=stride)))
    nx, ny, w, h, d = Y.shape
    return Y.reshape((nx * ny, w * h * d))

def callback(data):
    pub = rospy.Publisher('camera_prediction', Image, queue_size=10)
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv.resize(cv_image, (320, 240))

        X = prepare_image(cv_image, window=window, stride=stride)
        X = (X -  87.062)
        X = (X / 255.0)
        y_pred = model.predict(X)
        prediction_mask = (y_pred[:, 1].reshape(48, 64)*255).astype('uint8')

        # print(prediction_mask.max(), prediction_mask.min(), prediction_mask.mean())

        pub.publish(bridge.cv2_to_imgmsg(prediction_mask))
    except CvBridgeError as e:
        print(e)

def main():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('camera_predictor', anonymous=True)

    rospy.Subscriber('/sensors/camera/image', Image, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    main()
