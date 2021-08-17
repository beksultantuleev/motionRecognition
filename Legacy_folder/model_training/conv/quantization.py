import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import tensorflow as tf
from tensorflow import keras
import tensorflow_model_optimization as tfmot
import numpy as np

keras_model = keras.models.load_model("./conv_net.h5")

converter = tf.lite.TFLiteConverter.from_keras_model(keras_model)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
quantized = converter.convert()

open( "./lite_conv_net.tflite", 'wb').write(quantized)
