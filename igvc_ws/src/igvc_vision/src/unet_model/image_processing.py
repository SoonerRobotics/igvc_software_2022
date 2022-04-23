import cv2
import os
import numpy as np
import re

def readVideo(image, processing = None):
    # Load the image using OpenCV into numpy array

    # Process image
    if processing == 'in':

        # Histogram equalization of HSV value channel
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        image[:,:,2] = cv2.equalizeHist(image[:,:,2])
        image = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)

        image = cv2.transpose(image) / 255.0
        image = cv2.resize(image, (256, 256))

    if processing == 'out':
        image = cv2.transpose(image) / 255.0
        image = cv2.resize(image, (256, 256))

        image = image[:,:,0]

    return image

def post_process(image, threshold = None):
    if threshold is not None:
        _, image = cv2.threshold(image, threshold, 1, cv2.THRESH_BINARY)
    image = cv2.transpose(image * 255.0)
    image = cv2.resize(image, (1024, 576))
    return image