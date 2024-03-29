import cv2
import os
import numpy as np
import re

# Read a single PNG file
def readPNG(filename, processing = None):
    # Load the image using OpenCV into numpy array
    image = cv2.imread(filename)

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

def post_process(image, threshold = .2):
    if threshold is not None:
        _, image = cv2.threshold(image, threshold, 1, cv2.THRESH_BINARY)
    image = cv2.transpose(image)
    image = image * 255
    image = cv2.resize(image, (1024, 576))
    return image

# Read a set of PNG files
def read_images_from_directory(directory, file_regexp, whitelist = None, processing = None):
    print("Reading from ", directory, file_regexp)

    files = sorted(os.listdir(directory))

    if whitelist is None:
        list_of_images = [readPNG(directory + "/" + f, processing) for f in files if re.search(file_regexp, f)]
        file_names = [f for f in files if re.search(file_regexp, f)]
    else:
        list_of_images = [readPNG(directory + "/" + f, processing) for f in files if re.search(file_regexp, f) and f in whitelist]
        file_names = [f for f in files if re.search(file_regexp, f) and f in whitelist]

    print(list_of_images)
    return np.array(list_of_images, dtype=np.float32), file_names

