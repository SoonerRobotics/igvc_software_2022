import pickle
import numpy as np
import tensorflow as tf
from tensorflow import keras
import cv2

from image_processing import *
from unet import *

# Load the images
#ins, ins_filenames = read_images_from_directory("./example/video_frames", "img_[\d]*.png", processing='in')
cam = cv2.VideoCapture('test3.mp4')
ret, img = cam.read()
#print (ret)
ins = []
count = 0
while ret:
    ret, img = cam.read()
    cv2.imwrite(f"./example/video_in/f{count:04}.png", cv2.resize(img, (1024, 576)))
    img = readVideo(img, processing='in')
    
    ##cv2.imshow('before',img)
    if not ret:
            break
    #cv2.imshow('before', img)
    

    count = count + 1
    ins.append(img)
cam.release()
#print(ins)
ins = np.array(ins, dtype=np.float32)

# Reload the model
model = tf.keras.models.load_model('./results/SCRUNet_model')

# Make the predictions
predictions = model.predict(ins)

#fourcc = cv2.VideoWriter_fourcc(*'h264')
#out = cv2.VideoWriter('output.mp4', fourcc, 30.0, (1024,576))
out = cv2.VideoWriter('outpy.avi',cv2.VideoWriter_fourcc('M','J','P','G'), 30, (1024,576))

for i, prediction in enumerate(predictions):
    # Post process predictions to make images we can use
    prediction = post_process(prediction, threshold=0.5)
    #print(prediction)
    #out.write(prediction)
    cv2.imwrite(f"./example/video_prediction/f{i:04}.png", prediction)
out.release()