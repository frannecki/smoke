#!/usr/bin/env python
import smoke
import rospy
import os,sys
import numpy as np
import cv2 as cv
import keras
import tensorflow as tf
from smoke.srv import *
from keras import backend as K
from keras.models import load_model
from cv_bridge import CvBridge, CvBridgeError
import keras.backend.tensorflow_backend as KTF

class smoke_nn_server():

    def __init__(self):
        # print('{}/../models/nn/model.24.h5'.format(sys.path[0]))
        KTF.set_session(tf.Session(config=tf.ConfigProto(device_count={'cpu':0})))
        self.model = load_model('{}/../models/nn/model.24.h5'.format(sys.path[0]))
        self.bridge = CvBridge()
        print('model loaded')
        self.graph = tf.get_default_graph()
        rospy.init_node('smoke_nn_server')
        topicname = rospy.get_param('/smoke/services/image_svm_nn_srv/name')
        rospy.Service(topicname, darknet_svm_node, self.callback_nn)
        print ('Ready to monitor.')
        rospy.spin()

    def pred(self, arr):
        if K.image_data_format() == 'channels_first':
            arr = arr.transpose((0, 3, 1, 2))
        res = []
        with self.graph.as_default():
            pre = self.model.predict_classes(arr)
            for pred in enumerate(pre):
                res.append(np.argmax(pred))
        return res

    def callback_nn(self, req):
        cv_image = self.bridge.imgmsg_to_cv2(req.img, 'bgr8')
        bboxset = req.bboxes.bounding_boxes
        arr = []
        # rospy.loginfo('[smoke_nn_server] Received {} suspected bounding boxes'.format(len(bboxset)))
        for bbox in bboxset:
            xmin = bbox.xmin
            xmax = bbox.xmax
            ymin = bbox.ymin
            ymax = bbox.ymax
            bimg = cv_image[xmin:xmax, ymin:ymax, :]
            bimg_scale = cv.resize(bimg, (100, 100), cv.INTER_NEAREST)
            arr.append(bimg_scale)
        arr = np.asarray(arr)
        res = self.pred(arr) if arr.shape[0] > 0 else []
        return darknet_svm_nodeResponse(res)


if __name__ == "__main__":
    smoke_nn_server()
