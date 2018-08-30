#!/usr/bin/env python

import rospy, socket, struct
import cv2 as cv
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from skimage.util.shape import view_as_windows
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from smely_zajko_dataset import models

HOST = "localhost"
PORT = 8111

bridge = CvBridge()

window = (5, 5)
stride = 5

TRIANGLE_HEIGHT = 34
TRIANGLE_WIDTH = 6

model = models.mlp(n_input=75, architecture=[(20, 'sigmoid'), (2, 'softmax')],
                   metrics=['accuracy'])
model.load_weights(
    '/home/zajko/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/mlp_20_sigmoid_2_softmax.hdf5')


def prepare_image(image, window, stride):
    window_x, window_y = window
    Y = np.asarray(np.squeeze(view_as_windows(image,
                                              (window_x, window_y, 3),
                                              step=stride)))
    nx, ny, w, h, d = Y.shape
    return Y.reshape((nx * ny, w * h * d))


def callback(cv_image):
    pub = rospy.Publisher('camera_prediction', Image, queue_size=10)
    pub_triangle = rospy.Publisher('camera_triangles_prediction', Float64MultiArray, queue_size=10)
    try:
        cv_image = cv.resize(cv_image, (320, 240))

        X = prepare_image(cv_image, window=window, stride=stride)
        # X = (X - 87.062)
        X = (X / 255.0)
        y_pred = model.predict(X)
        prediction_mask = (y_pred[:, 1].reshape(48, 64) * 255).astype('uint8')

        # print(prediction_mask.max(), prediction_mask.min(), prediction_mask.mean())
    
	
	out = []
	for i in range(2, 64, TRIANGLE_WIDTH):
	    out.append(0)
	    s = 0
	    for j in range(47, TRIANGLE_HEIGHT, -1):
		b = int(j * TRIANGLE_WIDTH/TRIANGLE_HEIGHT)
		for k in range(i-b/2, i+b/2):
		    if k < 0 or k > 63:
			continue
		    out[-1] += prediction_mask[j][k]
		    s += 1
	    out[-1] /= s

	int_list = Float64MultiArray()
	int_list.data = out
	pub_triangle.publish(int_list)
        pub.publish(bridge.cv2_to_imgmsg(prediction_mask))
    except CvBridgeError as e:
        print(e)


def recv_msg(sock):
    raw_len = sock.recv(4)
    if not raw_len:
        return None, None
    msg_len = struct.unpack('>I', raw_len)[0]
    return msg_len, recv_n(sock, msg_len)

def recv_n(sock, n):
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def main():
    rospy.init_node('camera_predictor', anonymous=True)

#    rospy.Subscriber('/sensors/camera/image', Image, callback)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))

    while not rospy.is_shutdown():
        l, m = recv_msg(sock)
        img = cv.imdecode(np.fromstring(m, np.uint8), cv.IMREAD_COLOR)
        callback(img)
        cv.imshow('camera', img)
        cv.waitKey(1)

if __name__ == '__main__':
    main()


