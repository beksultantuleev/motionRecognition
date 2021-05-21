import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import tensorflow as tf
from tensorflow import keras
import tensorflow_model_optimization as tfmot
import numpy as np

keras_model = keras.models.load_model("./conv_net.h5")
train_data = np.load("training_data_2Dconv.npy")
train_labels = np.load("training_labels_2Dconv.npy")
test_data = np.load("test_data_2Dconv.npy")
test_labels = np.load("test_labels_2Dconv.npy")

def apply_pruning_to_dense(layer):
    if isinstance(layer, tf.keras.layers.Dense):
        return tfmot.sparsity.keras.prune_low_magnitude(layer)
    return layer

model_for_pruning = tf.keras.models.clone_model( keras_model, clone_function= apply_pruning_to_dense)

batch_size = 10
epochs = 15
validation_split = 0.2

model_for_pruning.compile(optimizer='adam', loss=tf.keras.losses.CategoricalCrossentropy(), metrics =['accuracy'])
model_for_pruning.summary()

callbacks= [ tfmot.sparsity.keras.UpdatePruningStep()]

model_for_pruning.fit(train_data, train_labels, batch_size= batch_size, epochs = epochs, validation_split=validation_split, callbacks= callbacks, verbose =2)

pruned_score = model_for_pruning.evaluate(test_data,test_labels,verbose=2)

results_names = model_for_pruning.metrics_names
result = "\nAfter pruning the %s value is: %f \nAfter pruning the %s value is: %f \n" %(results_names[0] ,pruned_score[0],results_names[1] ,pruned_score[1])
print(result)

model_for_export = tfmot.sparsity.keras.strip_pruning(model_for_pruning)

converter = tf.lite.TFLiteConverter.from_keras_model(model_for_export)
converter.optimizations = [tf.lite.Optimize.DEFAULT]
quantized_and_pruned = converter.convert()

open( "./lite_conv_net_pruned.tflite", 'wb').write(quantized_and_pruned)
open( "./training_info.txt", 'wb').write(result)
