#!/usr/bin/env python

import rospy, socket, struct, re
import cv2 as cv
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64MultiArray, UInt8
from std_msgs.msg import UInt8MultiArray
from skimage.util.shape import view_as_windows
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from smely_zajko_dataset import models
import pyzbar.pyzbar as zbar
from ctypes import *
import numpy.ctypeslib as npct


HOST = "192.168.42.129"
#HOST = "localhost"
PORT = 8000

bridge = CvBridge()

window = (5, 5)
stride = 5

TRIANGLE_HEIGHT = 34
TRIANGLE_WIDTH = 6

pub_eval = 0
counterr = 0

model = models.mlp(n_input=75, architecture=[(100, 'relu'), (100, 'relu'), (1, 'sigmoid')],
                   metrics=['accuracy'])

#model = models.mlp(n_input=75, architecture=[(80, 'sigmoid'), (2, 'softmax')],
#                   metrics=['accuracy'])

#model = models.mlp(n_input=75, architecture=[(100, 'sigmoid'), (100, 'sigmoid'), (2, 'softmax')],
#                   metrics=['accuracy'])



#model.load_weights(
#    '/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/mlp_20_sigmoid_2_softmax.hdf5')

#model.load_weights(
#    '/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/hsv/mlp_cat_100_sig_100_sig_2_softmax.h5')

#model.load_weights(
#    '/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/hsv/mlp_cat_80_sig_2_softmax.h5')

model.load_weights('/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/wednesday/mlp_100_relu_100_relu_1_sig.h5')


hsvlib = npct.load_library("librgb2hsv.so", "/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts")
array_3d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=3, flags=('CONTIGUOUS','WRITEABLE'))
array_2d_float64 = npct.ndpointer(dtype=np.float64, ndim=2, flags=('CONTIGUOUS','WRITEABLE'))
array_2d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=2, flags=('CONTIGUOUS','WRITEABLE'))
array_1d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=1, flags=('CONTIGUOUS','WRITEABLE'))
hsvlib.rgb2hsv.argtypes = [array_3d_uint8t, c_int, c_int]
hsvlib.init.argtypes = [array_3d_uint8t]
hsvlib.evaluate.argtypes = [array_3d_uint8t, array_2d_uint8t, c_int, c_int]
positive_img = cv.imread("/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/positive.png");
hsvlib.init(positive_img)
evaluated_img = np.zeros((60,60), np.uint8)

def prepare_image(image, window, stride):
    window_x, window_y = window
    Y = np.asarray(np.squeeze(view_as_windows(image,
                                              (window_x, window_y, 3),
                                              step=stride)))
    nx, ny, w, h, d = Y.shape
    return Y.reshape((nx * ny, w * h * d))


def callback(cv_image):
    global counterr
    cv_image = cv.flip(cv_image, -1)
    pub = rospy.Publisher('camera_prediction', Image, queue_size=1)
    pub_triangle = rospy.Publisher('camera_triangles_prediction', Float64MultiArray, queue_size=1)
    pub_eval = rospy.Publisher('/sensors/camera/evaluated_image', UInt8MultiArray, queue_size=3)
    try:
        cv_image = cv.resize(cv_image, (640, 480))
        
        cvi_height, cvi_width, cvi_channels = cv_image.shape
        hsvlib.rgb2hsv(cv_image, cvi_width, cvi_height)
        hsvlib.evaluate(cv_image, evaluated_img, cvi_width, cvi_height)
        
        print("publishing evaluated image" + str(counterr))
        counterr += 1
        eval_msg = UInt8MultiArray()

        evalout = []
        for i in range(0, 60):
          for j in range(0, 60):
            evalout.append(evaluated_img[i][j])

        eval_msg.data = evalout
        pub_eval.publish(eval_msg)
     
        print("published evaluated image")

        X = prepare_image(cv_image, window=window, stride=stride)
        # X = (X - 87.062)
        X = (X / 255.0)
        y_pred = model.predict(X)
        #prediction_mask = (255 - (y_pred[:, 1].reshape(96, 128) * 255)).astype('uint8')
        prediction_mask = (y_pred.reshape(96, 128) * 255).astype('uint8')

        # nove:
        # prediction_mask =  (y_pred.reshape(96, 128) * 255).astype('uint8')

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

QR_pattern = re.compile('geo:([-+]?\d*\.\d+|[-+]?\d+),([-+]?\d*\.\d+|[-+]?\d+)')

def detectQR(img):
    pub = rospy.Publisher('/control/camera_qr_target', NavSatFix, queue_size=10)

    decoded = zbar.decode(img)
    for obj in decoded:
        if obj.type == 'QRCODE':
            match = QR_pattern.match(obj.data)
            if match:
                target = NavSatFix()
                target.latitude = float(match.group(1))
                target.longitude = float(match.group(2))
                pub.publish(target)
                return # stop evaluating other qr codes when correct one was found

action = 0

def setAction(act):
    global action
    action = act.data

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
    global pub_eval
    rospy.init_node('camera_predictor', anonymous=True)
    pub_eval = rospy.Publisher('/sensors/camera/evaluated_image', UInt8MultiArray, queue_size=3)

#    rospy.Subscriber('/sensors/camera/image', Image, callback)
    rospy.Subscriber('/control/camera_action', UInt8, setAction)

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect((HOST, PORT))
    print "Connected to camera"
    
    while not rospy.is_shutdown():
        l, m = recv_msg(sock)
#        print l
        img = cv.imdecode(np.fromstring(m, np.uint8), cv.IMREAD_COLOR)
        callback(img)
        if action == 1:
            detectQR(img)

        #cv.imshow('camera', img)
        #cv.waitKey(1)

if __name__ == '__main__':
    main()


