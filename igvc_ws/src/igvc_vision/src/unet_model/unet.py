import tensorflow as tf
from tensorflow import keras
from tensorflow.keras import datasets, layers, models

def unet():
    inp = layers.Input(shape=(256, 256, 3))

    # Convolution layers to help learn some basic kernels
    down_samples = [0] * 4
    down_samples[0] = layers.Conv2D(8, (3, 3), strides=(1, 1), padding='same', activation='relu')(inp) 
    # Down sampling
    for x in range(1,3):
        down_samples[x] = layers.Conv2D(16, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(down_samples[x-1])
    # Most compressed layer in the network
    print(down_samples)
    latent = layers.Conv2D(16, kernel_size=(3, 3), strides=(2,2), padding='same', activation='relu')(down_samples[3])

    # Upsampling with skip connections
    up_samples =  [0] * 4
    skip = []
    up_samples[0] = layers.Conv2DTranspose(24, (3, 3), strides=(2,2), padding='same', activation='relu')(latent)
    skip[0] = layers.Concatenate()([up_samples[0], down_samples[3]])
    
    # Down sampling
    for x in range(1, 3):
        up_samples[x] = layers.Conv2DTranspose(24, (3, 3), strides=(2,2), padding='same', activation='relu')(skip[x-1]) 
        skip[x] = layers.Concatenate()([x, down_samples[3-x]])

    # Post convolution layers
    post_conv = layers.Conv2DTranspose(8, (3, 3), strides=(1, 1), padding='same', activation='relu')(skip[3])
    output = layers.Conv2DTranspose(1, (1, 1), strides=(1,1), padding='same', activation='sigmoid')(post_conv)

    model = models.Model(inputs=inp, outputs=output)

    # Bind the optimizer and the loss function to the model
    model.compile(optimizer='adam',
              loss=tf.keras.losses.BinaryCrossentropy(),
              metrics=[tf.keras.metrics.BinaryAccuracy()])

    return model