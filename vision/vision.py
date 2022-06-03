#Vision.py
import cv2
import numpy as np


def process_frame(img):
    old = img

    img = cv2.GaussianBlur(img, (7,7), 0)
    img = cv2.GaussianBlur(img, (7,7), 0)
    blank_mask = np.zeros(img.shape, dtype=np.uint8)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #remove shadow
    lower = np.array([60, 0, 0])
    upper = np.array([128, 255, 125])
    smask = cv2.inRange(hsv, lower, upper)

    #ramp mask subtract
    # create a lower bound for a pixel value
    # create an upper bound for a pixel values
    lower = np.array([23, 0, 0])
    upper = np.array([158, 41, 249])
    rmask = cv2.inRange(hsv, lower, upper)

    # subtraction mask that removes the lip and shadow around the ramp
    lower = np.array([0, 0, 0])
    upper = np.array([0, 255, 255])
    umask = cv2.inRange(hsv, lower, upper)

    #lmask additive
    # create a lower bound for a pixel value
    # create an upper bound for a pixel values
    # detects all white pixels wihin the range specified earlier
    lower = np.array([0, 0, 207])
    upper = np.array([255, 82, 255])
    lmask = cv2.inRange(hsv, lower, upper)
    #redmask addititve
    lower = np.array([0, 127, 0])
    upper = np.array([179, 255, 255])
    redmask = cv2.inRange(hsv, lower, upper)


    #gmask = gmask1 - gmask2
    #smask = 255 - smask
    lower = np.array([0, 127, 0])
    upper = np.array([0, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)

    mask = lmask - rmask - umask - smask + redmask
    #smask = cv2.medianBlur(mask, 7)
    return mask

                          
                                # press q to quit
def crop(img):
    crop = img[0:460,0:960 ]
    return crop
    #out.release()
cam = cv2.VideoCapture('test3.mp4')
ret, img = cam.read()
count = 0
while ret:
    ret, img = cam.read()
    if not ret:
            break
    img = cv2.resize(img, (960, 540))
    img = crop(img)
    img = cv2.resize(img, (960, 540))
    cv2.imwrite(f"./in/f{count:04}.png", img)
    count = count + 1  
    cv2.imshow('before',img)
    img2 = process_frame(img)
    cv2.imshow("final",img2)
    cv2.imwrite(f"./out/f{count:04}.png", img2)
    if cv2.waitKey(1) & 0xFF == ord('q'): # btw, you need to click the screen first. And then 
        break  
cam.release()
cv2.waitKey(0)
cv2.destroyAllWindows()