__author__ = 'zhengwang'

import cv2
import numpy as np
import glob
import sys
import time
import os
from sklearn.model_selection import train_test_split


def load_data(input_size, path):
    print("Loading training data...")
    start = time.time()

    # load training data
    X = np.empty((0, input_size))
    y = np.empty((0, 2))
    # training_data get list of files (*.npz).
    #['training_data/1586354010.npz', 'training_data/1586353551.npz', 'training_data/1586353305.npz', 'training_data/1586354363.npz', 'training_data/1586352846.npz', 'training_data/1586353682.npz', 'training_data/1586353488.npz', 'training_data/1586353207.npz', 'training_data/1586353094.npz', 'training_data/1586352905.npz', 'training_data/1586354493.npz', 'training_data/1586354594.npz', 'training_data/1586353812.npz', 'training_data/1586353025.npz', 'training_data/1586354264.npz', 'training_data/1586353883.npz', 'training_data/1586354436.npz', 'training_data/1586354080.npz', 'training_data/1586354653.npz', 'training_data/1586354743.npz']
    training_data = glob.glob(path)

    # if no data, exit
    if not training_data:
        print("Data not found, exit")
        sys.exit()

    for single_npz in training_data: # single_npz variable has string of name of file.
        # signle_npx => 'training_data/1586354010.npz'
        with np.load(single_npz) as data:
            # data has 2 np array files.
            # print(data.files)  # ['train', 'train_labels']
            train = data['train']
            # train is one np array having camera image. 
            train_labels = data['train_labels']
            # train_label is one np array having little cogit car direction code. 
        X = np.vstack((X, train))
        y = np.vstack((y, train_labels))

    print("Image array shape: ", X.shape)
    print("Label array shape: ", y.shape)

    end = time.time()
    print("Loading data duration: %.2fs" % (end - start))

    # normalize data => Each elemnt of X has range 0 ~ 255. Normailze: 0 ~255 -> 0 ~ 1 
    X = X / 255.

    # train validation split, 7:3
    return train_test_split(X, y, test_size=0.3)


class NeuralNetwork(object):
    def __init__(self):
        self.model = None

    def create(self, layer_sizes):
        # create neural network
        self.model = cv2.ml.ANN_MLP_create()
        self.model.setLayerSizes(np.int32(layer_sizes))
        #self.model.setTrainMethod(cv2.ml.ANN_MLP_BACKPROP)
        self.model.setTrainMethod(cv2.ml.ANN_MLP_RPROP)
        self.model.setActivationFunction(cv2.ml.ANN_MLP_SIGMOID_SYM, 2, 1)
        #self.model.setActivationFunction(cv2.ml.ANN_MLP_LEAKYRELU, 2, 1)
        self.model.setTermCriteria((cv2.TERM_CRITERIA_COUNT, 100, 0.01))

    def train(self, X, y):
        # set start time
        start = time.time()

        print("Training ...")
        self.model.train(np.float32(X), cv2.ml.ROW_SAMPLE, np.float32(y))

        # set end time
        end = time.time()
        print("Training duration: %.2fs" % (end - start))

    def evaluate(self, X, y):
        ret, resp = self.model.predict(X)
        prediction = resp.argmax(-1)
        true_labels = y.argmax(-1)
        accuracy = np.mean(prediction == true_labels)
        return accuracy

    def save_model(self, path):  
        directory = "saved_model"
        if not os.path.exists(directory):
            os.makedirs(directory)
        self.model.save(path)
        print("Model saved to: " + "'" + path + "'")

    def load_model(self, path):
        if not os.path.exists(path):
            print("Model does not exist, exit")
            sys.exit()
        self.model = cv2.ml.ANN_MLP_load(path)

    def predict(self, X):
        resp = None
        try:
            ret, resp = self.model.predict(X)
        except Exception as e:
            print(e)
        return resp.argmax(-1)
