#!/usr/bin/env python

import rospy, socket, struct, re
import cv2 as cv
from sensor_msgs.msg import Image, NavSatFix
from std_msgs.msg import Float64MultiArray, UInt8
from skimage.util.shape import view_as_windows
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from smely_zajko_dataset import models
import pyzbar.pyzbar as zbar

HOST = "192.168.42.129"
#HOST = "localhost"
PORT = 8000

bridge = CvBridge()

window = (5, 5)
stride = 5

TRIANGLE_HEIGHT = 34
TRIANGLE_WIDTH = 6

model = models.mlp(n_input=75, architecture=[(80, 'sigmoid'), (2, 'softmax')],
                   metrics=['accuracy'])

#model = models.mlp(n_input=75, architecture=[(100, 'sigmoid'), (100, 'sigmoid'), (2, 'softmax')],
#                   metrics=['accuracy'])



#model.load_weights(
#    '/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/mlp_20_sigmoid_2_softmax.hdf5')

#model.load_weights(
#    '/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/hsv/mlp_cat_100_sig_100_sig_2_softmax.h5')

model.load_weights(
    '/home/nvidia/Projects/smely-zajko-ros/src/robot_control/src/scripts/smely_zajko_dataset/hsv/mlp_cat_80_sig_2_softmax.h5')


def rgb_to_hsv(r, g, b):
  if r > g:
    if r > b:
      rgb_max = r
      if g > b:
        rgb_min = b
      else:
        rgb_min = g
    else:
      rgb_max = b
      rgb_min = g
  elif g > b:
    rgb_max = g
    if r > b:
      rgb_min = b
    else:
      rgb_min = r
  else:
    rgb_max = b
    rgb_min = r
  rgb_c = rgb_max - rgb_min
  if rgb_c == 0:
    h = 0
  elif rgb_max == r:
    h = ((g - b) / rgb_c) * 42.5
  elif rgb_max == g:
    h = ((b - r) / rgb_c + 2) * 42.5
  else:
    h = ((r - g) / rgb_c + 4) * 42.5
  if h < 0:
    h += 255
  if rgb_max == 0:
    s = 0
  else:
    s = 255 * (rgb_max - rgb_min) / rgb_max
  v = rgb_max
  return h, s, v

        

def prepare_image(image, window, stride):
    window_x, window_y = window
    Y = np.asarray(np.squeeze(view_as_windows(image,
                                              (window_x, window_y, 3),
                                              step=stride)))
    nx, ny, w, h, d = Y.shape
    return Y.reshape((nx * ny, w * h * d))


def callback(cv_image):
    cv_image = cv.flip(cv_image, -1)
    pub = rospy.Publisher('camera_prediction', Image, queue_size=10)
    pub_triangle = rospy.Publisher('camera_triangles_prediction', Float64MultiArray, queue_size=10)
    try:
        cv_image = cv.resize(cv_image, (640, 480))
        for x in range(0, 480):
            for y in range(0, 640):
                cv_image[x,y,0], cv_image[x,y,1], cv_image[x,y,2] = rgb_to_hsv(cv_image[x,y,0], cv_image[x,y,1], cv_image[x,y,2])
        X = prepare_image(cv_image, window=window, stride=stride)
        # X = (X - 87.062)
        X = (X / 255.0)
        y_pred = model.predict(X)
        prediction_mask = ((y_pred[:, 1].reshape(96, 128) * 255)).astype('uint8')

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
    rospy.init_node('camera_predictor', anonymous=True)

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


