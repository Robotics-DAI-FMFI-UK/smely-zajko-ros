#!/usr/bin/env python

import time
import datetime
import threading
import os
import socket, struct, re

import cv2 as cv
from skimage.util.shape import view_as_windows
import numpy as np
#from smely_zajko_dataset import models
import pyzbar.pyzbar as zbar
from ctypes import *
import numpy.ctypeslib as npct

import resnet_model as resnet


ANDROID_HOST = "192.168.42.129"
ANDROID_PORT = 8000

ZAJKO_HOST = "169.254.0.100"
ZAJKO_PORT = 9771
QR_PORT = 9770

window = (5, 5)
stride = 5

NEURON_IN_USE = True

LOG_PREDICTIONS = True

now = datetime.datetime.now()
LOG_FOLDER = '/media/nvidia/UBUNTU 18_0/camera_log/' + now.strftime('%Y-%m-%d-%H%M%S')

counterr = 0
running = True
sockZ = None
sockQ = None

QR_pattern = re.compile('geo:([-+]?\d*\.\d+|[-+]?\d+),([-+]?\d*\.\d+|[-+]?\d+)')

RED = np.ones((480, 640, 3), dtype=np.uint8) * (0, 0, 1)
WHITE = np.ones((480, 640, 3), dtype=np.uint8) * (1, 1, 1)
TRANSPARENCY = 0.9

# TEST SEGMENTACNEJ NEURONKY
start_loading = time.time()
model = resnet.resnet()
end_loading = time.time()
print('Loading resnet.resnet() took {} sec'.format(end_loading - start_loading))

start_loading = time.time()
#model.load_weights('resnet.hdf5')
model.load_weights('resnet_night.hdf5')
end_loading = time.time()
print('Loading weights took {} sec'.format(end_loading - start_loading))

#from keras.models import load_model
#start_prediction = time.time()
#model = load_model('./resnet.hdf5', compile=False)
#model = load_model('./resnet_night.hdf5')
#end_prediction = time.time()
#print('Loading model took {} sec'.format(end_prediction - start_prediction))

# model_json = model.to_json()
# with open("resnet_model.json", 'w') as json_file:
#     json_file.write(model_json)
# 
# model.save_weights("resnet_model.h5")

# import tensorflow as tf
# from keras.backend.tensorflow_backend import set_session
# config = tf.ConfigProto()
# config.intra_op_parallelism_threads = 6
# config.inter_op_parallelism_threads = 6
# set_session(tf.Session(config=config))


# print('Started with loading model')
# from keras.models import model_from_json
# start_prediction = time.time()
# model = model_from_json(open('./resnet_model.json').read())
# end_prediction = time.time()
# print('Loading model from json took {} sec'.format(end_prediction - start_prediction))
# 
# start_prediction = time.time()
# model.load_weights('./resnet_model.h5')
# end_prediction = time.time()
# print('Loading model weights took {} sec'.format(end_prediction - start_prediction))

hsvlib = npct.load_library("librgb2hsv.so", "/home/nvidia/src/smely-zajko-ros/jetson-camera/scripts")
array_3d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=3, flags=('CONTIGUOUS','WRITEABLE'))
array_2d_float64 = npct.ndpointer(dtype=np.float64, ndim=2, flags=('CONTIGUOUS','WRITEABLE'))
array_2d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=2, flags=('CONTIGUOUS','WRITEABLE'))
array_1d_uint8t = npct.ndpointer(dtype=np.ubyte, ndim=1, flags=('CONTIGUOUS','WRITEABLE'))
array_1d_float64 = npct.ndpointer(dtype=np.float64, ndim=1, flags=('CONTIGUOUS','WRITEABLE'))
hsvlib.rgb2hsv.argtypes = [array_3d_uint8t, c_int, c_int]
hsvlib.init.argtypes = [array_3d_uint8t]
hsvlib.init_triangle.argtypes = [array_3d_uint8t]
hsvlib.evaluate.argtypes = [array_3d_uint8t, array_2d_uint8t, c_int, c_int]
hsvlib.evaluate_triangles.argtypes = [array_3d_uint8t, array_2d_uint8t, array_1d_float64, c_int, c_int]
positive_img = cv.imread("/home/nvidia/src/smely-zajko-ros/src/robot_control/src/scripts/positive.png")
hsvlib.init(positive_img)
hsvlib.init_triangle(positive_img)
evaluated_img = np.zeros((60,60), np.uint8)
eval_triangles = np.zeros((11,), np.float64)
posimage = np.zeros((480,640), np.uint8)


# Funkcia na predpripravu obrazku pre staru neuronku
def prepare_image(image, window, stride):
    window_x, window_y = window
    Y = np.asarray(np.squeeze(view_as_windows(image,
                                              (window_x, window_y, 3),
                                              step=stride)))
    nx, ny, w, h, d = Y.shape
    return Y.reshape((nx * ny, w * h * d))



def callback(cv_image):
    global counterr, sockZ

    # flip image because android phone is rotated
    cv_image = cv.flip(cv_image, -1)
    cv_image = cv.resize(cv_image, (640, 480))

    cvi_height, cvi_width, cvi_channels = cv_image.shape
    ###hsvlib.rgb2hsv(cv_image, cvi_width, cvi_height)
    ###hsvlib.evaluate(cv_image, evaluated_img, cvi_width, cvi_height)
        
    if NEURON_IN_USE:
        ###X = prepare_image(cv_image, window=window, stride=stride)
        ###X = (X - 87.062)
        ###X = (X / 255.0)
        cv_image_converted = cv.cvtColor(cv_image, cv.COLOR_BGR2RGB)
        X = cv_image_converted / 255

        start_prediction = time.time()
        y_pred = model.predict(np.expand_dims(X, axis=0))
        end_prediction = time.time()

        print('Predicted in {} sec'.format(end_prediction - start_prediction))

        y_pred[y_pred <= 0.5] = 0
        y_pred[y_pred > 0.5] = 255

        prediction_mask = y_pred[0].astype(np.uint8)
        
        ###prediction_mask = ((y_pred[:, 0].reshape(96, 128) * 255)).astype('uint8')
        #prediction_mask = (y_pred.reshape(96, 128) * 255).astype('uint8')
    
        # nove:
        # prediction_mask =  (y_pred.reshape(96, 128) * 255).astype('uint8')
    
        cvi_height, cvi_width, cvi_channels = prediction_mask.shape
        hsvlib.evaluate(prediction_mask, evaluated_img, cvi_width, cvi_height)

        # send prediction_mask and evaluate_img over socket to zajko
        eval_img_packet_header = b'\x14\x0e\x00\x00\x01\x00\x00\x00'
        try:
            sockZ.send(eval_img_packet_header, 8)
            sockZ.send(evaluated_img, 3600)
        except:
            print("problem sending data to zajko, disconnected?")
            try:
                sockZ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                print(sockZ.connect((ZAJKO_HOST, ZAJKO_PORT)))
                print("reconnected to localmap@zajko")
            except:
                print("reconnect failed")
                time.sleep(3)

        p = (RED * y_pred[0]).astype(np.uint8)

        out = cv.addWeighted(cv_image, 1, p, TRANSPARENCY, 0)

        cv.imshow('camera', out)
        cv.waitKey(1)

        if LOG_PREDICTIONS:
            name = '{}.png'.format(counterr)
            name_mask = '{}_mask.png'.format(counterr)
            try:
                cv.imwrite(os.path.join(LOG_FOLDER, name), cv_image)
                cv.imwrite(os.path.join(LOG_FOLDER, name_mask), prediction_mask)
            except:
                print('Cannot write image to disk. Check disk usage.')

        counterr += 1


def detectQR(img):
    global sockQ
    decoded = zbar.decode(img)
    for obj in decoded:
        if obj.type == 'QRCODE':
            match = QR_pattern.match(obj.data.decode())
            if match:
                #target = NavSatFix()
                #target.latitude = float(match.group(1))
                #target.longitude = float(match.group(2))

                latitude = match.group(1)
                longitude = match.group(2)
                print('GOT QR CODE lat: {}, lon: {}'.format(latitude, longitude))
                qr_packet = (latitude + " " + longitude).encode('ascii')
                len_packet = bytes([len(qr_packet),0,0,0])

                try:
                    sockQ.send(len_packet)
                    sockQ.send(qr_packet)
                    print("QR CODE sent")
                except Exception as e:
                    print(e)
                    print("problem sending qr data to zajko, disconnected?")
                    try:
                        sockQ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                        print(sockQ.connect((ZAJKO_HOST, QR_PORT)))
                        print("reconnected to qr@zajko")
                    except:
                        print("reconnect qr failed")
                        time.sleep(3)

                return # stop evaluating other qr codes when correct one was found
            else:
                print('QR PATTERN: {}'.format(obj.data.decode()))

action = 0

def setAction(act):
    global action
    action = act.data
    print(act)

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

def qr_thread():
    global action, sockQ
    while running:
        try:
            action = sockQ.recv(1)[0]
            print("action: " + str(action))
        except:
            try:
                if action == 0:
                    sockQ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                    print(sockQ.connect((ZAJKO_HOST, QR_PORT)))
                    print("reconnected to qr@zajko")
            except:
                print("reconnecting qr failed")
            time.sleep(1)


def main():
    global sockZ, sockQ
    print("Starting to connect...")

    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(sock.connect((ANDROID_HOST, ANDROID_PORT)))
    print("Connected to camera")
    
    sockZ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(sockZ.connect((ZAJKO_HOST, ZAJKO_PORT)))
    print("Connected to localmap@zajko")

    sockQ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print(sockQ.connect((ZAJKO_HOST, QR_PORT)))
    print("Connected to qr@zajko")

    thread_qr = threading.Thread(target = qr_thread, args = [])
    thread_qr.start()

    if LOG_PREDICTIONS:
        os.makedirs(LOG_FOLDER, exist_ok=True)


    while running:
        l, m = recv_msg(sock)
        print(l)
        img = cv.imdecode(np.fromstring(m, np.uint8), cv.IMREAD_COLOR)
        callback(img)

        if action == 1:
            detectQR(img)

if __name__ == '__main__':
    main()


