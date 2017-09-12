#!/usr/bin/env python

import rospy
import cv2 as cv
from sensor_msgs.msg import Image
from skimage.util.shape import view_as_windows
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from keras.models import load_model

bridge = CvBridge()

window = (5, 5)
stride = 5

model = load_model('/home/zajko/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/small_middle_cnn.hdf5')


def prepare_image(image, window, stride):
    window_x, window_y = window
    padd = map(int, np.ceil((window_x / 2.0, window_y / 2.0)))
    img = np.pad(image, (padd, padd, (0, 0)), 'constant',
                 constant_values=(0, 0))
    X = np.asarray(np.squeeze(view_as_windows(img,
                                              (window_x, window_y, 3),
                                              step=stride)))
    X = X[1:, 1:, :, :, :]
    nx, ny, w, h, d = X.shape
    return np.rollaxis(X.reshape((nx * ny, w, h, d)), 3, start=1)


def callback(data):
    pub = rospy.Publisher('camera_prediction', Image, queue_size=10)
    try:
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
        cv_image = cv.resize(cv_image, (320, 240))

        X = prepare_image(cv_image, window=window, stride=stride)
        X = (X - 87.062)
        X = (X / 255.0)
        y_pred = model.predict(X)
        prediction_mask = (np.reshape(y_pred, (48, 64)) * 255).astype('uint8')
        pub.publish(bridge.cv2_to_imgmsg(prediction_mask))
    except CvBridgeError as e:
        print(e)


def main():
    rospy.init_node('camera_predictor', anonymous=True)

    rospy.Subscriber('/sensors/camera/image', Image, callback)

    rospy.spin()


if __name__ == '__main__':
    main()
