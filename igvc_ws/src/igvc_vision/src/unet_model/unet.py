import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import datasets, layers, models

def unet():
    inp = layers.Input(shape=(256, 256, 3))

    # Convolution layers to help learn some basic kernels
    down0 = layers.Conv2D(8, (3, 3), strides=(1, 1), padding='same', activation='relu')(inp) 
    pool1 = tf.keras.layers.MaxPooling2D(pool_size = (2,2))(down0)

    down1 = layers.Conv2D(16, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(pool1)
    pool2 = tf.keras.layers.MaxPooling2D(pool_size = (2,2))(down1)

    down2 = layers.Conv2D(24, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(pool2)
    pool3 = tf.keras.layers.MaxPooling2D(pool_size = (2,2))(down2)

    down3 = layers.Conv2D(24, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(pool3)
    pool4 = tf.keras.layers.MaxPooling2D(pool_size = (2,2))(down3)

    #up sampling
    up0 = layers.Conv2DTranspose(24, (3, 3), strides=(2,2), padding='same', activation='relu')(pool4)    
    up0 = layers.UpSampling2D(size=(4,4)) (up0)
    skip0 = layers.Concatenate()([up0, down2])

    up1 = layers.Conv2DTranspose(24, (3, 3), strides=(2,2), padding='same', activation='relu')(skip0)
    up1 = layers.UpSampling2D(size=(2,2)) (up1)
    skip1 = layers.Concatenate()([up1, down1])

    up2 = layers.Conv2DTranspose(16, (3, 3), strides=(2,2), padding='same', activation='relu')(skip1)
    up2 = layers.UpSampling2D(size=(2,2)) (up2)
    skip2 = layers.Concatenate()([up2, down0])
    up3 = layers.Conv2DTranspose(8, (3, 3), strides=(1,1), padding='same', activation='relu')(skip2)

    # Post convolution layers
    output = layers.Conv2DTranspose(1, (1, 1), strides=(1,1), padding='same', activation='sigmoid')(up3)
    #output = up3
    model = models.Model(inputs=inp, outputs=output)

    # Bind the optimizer and the loss function to the model
    model.compile(optimizer='adam',
              loss=tf.keras.losses.BinaryCrossentropy(),
              metrics=[tf.keras.metrics.BinaryAccuracy()])

    return model