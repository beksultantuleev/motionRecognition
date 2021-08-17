import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import pandas as pd
import numpy as np
import tensorflow_model_optimization as tfmot
import re

from keras.models import Sequential
from keras.layers import Dense, Conv2D, MaxPooling2D, Flatten, Dropout
from keras import optimizers
from keras.utils import to_categorical


''' Main Code '''
# Clear the dataset from while lines and text, save the last acquisition number
format = re.compile('\d+\,[AEIOU]\,-?(\d+)\,-?(\d+)\,-?(\d+)')
# print(re.search(format,'1,a,123,-15,-2566').group()) example on data format

log_file_path = "/home/es_20-21/Desktop/Project/02-Custom_NN/dataset/log_file.txt"
dataset_path = "/home/es_20-21/Desktop/Project/02-Custom_NN/dataset/dataset.txt"
with open(log_file_path,'r') as data_file:
    with open(dataset_path,'w') as out_file:
        for counter,line in enumerate(data_file):
            if re.search(format,line):
                out_file.write(line)

# Now use panda to handle the dataset
columnNames = ['acquisition','letter','ax','ay','az']
dataset = pd.read_csv(dataset_path,header = None, names=columnNames,na_values=',')
# find the number
last_index = max(np.unique(dataset.acquisition))

second_axis = [];
for acq_index in range(2,last_index):
    second_axis.append(dataset[dataset.acquisition == acq_index].shape[0])
print(min(second_axis))

tensor = np.empty((0,min(second_axis),3))
print(tensor.shape)
labels= np.empty((0))

#SOLVE THA FACT THE VALUE 1 IS NOT TAKEN
# REMEMBER TO RANDOMIZE
for acq_index in range(2,last_index):
    temp = dataset[dataset.acquisition == acq_index]
    ax = temp.ax
    ay = temp.ay
    az = temp.az
    tensor = np.vstack([tensor,np.dstack([ax, ay, az])])
    labels = np.append(labels,np.unique(temp.letter))

#print(type(labels[1]))
labels = np.asarray(pd.get_dummies(labels),dtype = np.int8)
#print(labels)
#print(to_categorical(labels))
numOfRows = tensor.shape[1]
numOfColumns = tensor.shape[2]
numChannels = 1
numFilters = 32 # number of filters in Conv2D layer
# kernal size of the Conv2D layer
kernalSize1 = 2
# max pooling window size
poolingWindowSz = 2
# number of filters in fully connected layers
numNueronsFCL1 = 128 #128
numNueronsFCL2 = 64 #128
# split ratio for test and validation
trainSplitRatio = 0.8
# number of epochs
Epochs = 15 # originally epochs was 10
# batchsize
batchSize = 10 #10
# number of total clases
numClasses = labels.shape[1]
# dropout ratio for dropout layer
dropOutRatio = 0.2
# reshaping the data for network input
reshapedSegments = tensor.reshape(tensor.shape[0], numOfRows, numOfColumns,1)
# splitting in training and testing data
trainSplit = np.random.rand(len(reshapedSegments)) < trainSplitRatio
trainX = reshapedSegments[trainSplit]
testX = reshapedSegments[~trainSplit]
trainX = np.nan_to_num(trainX)
testX = np.nan_to_num(testX)
trainY = labels[trainSplit]
testY = labels[~trainSplit]

model = Sequential()
# adding the first convolutionial layer with 32 filters and 5 by 5 kernal size, using the rectifier as the activation function
model.add(Conv2D(numFilters, (kernalSize1,kernalSize1),input_shape=(numOfRows, numOfColumns,1),activation='relu'))
# adding a maxpooling layer
model.add(MaxPooling2D(pool_size=(poolingWindowSz,poolingWindowSz),padding='valid'))
# adding a dropout layer for the regularization and avoiding over fitting
model.add(Dropout(dropOutRatio))
# flattening the output in order to apply the fully connected layer
model.add(Flatten())
# adding first fully connected layer with 256 outputs
model.add(Dense(numNueronsFCL1, activation='relu'))
#adding second fully connected layer 128 outputs
model.add(Dense(numNueronsFCL2, activation='relu'))
# adding softmax layer for the classification
model.add(Dense(numClasses, activation='softmax'))
# Compiling the model to generate a model
model.summary()
adam = optimizers.Adam(lr = 0.001, decay=1e-6)
model.compile(loss='categorical_crossentropy', optimizer=adam, metrics=['accuracy'])

for layer in model.layers:
    print(layer.name)

model.fit(trainX,trainY, validation_split=1-trainSplitRatio,epochs=Epochs,batch_size=batchSize,verbose=2)
score = model.evaluate(testX,testY,verbose=2)

results_names = model.metrics_names
result = "\nThe %s value is: %f \nThe %s value is: %f \n" %(results_names[0] ,score[0],results_names[1] ,score[1])
print(result)

model.save('./conv_net.h5')
open( "./training_info.txt", 'wb').write(result)
np.save('./training_data_2Dconv.npy', trainX)
np.save('./training_labels_2Dconv.npy', trainY)
np.save('./test_data_2Dconv.npy', testX)
np.save('./test_labels_2Dconv.npy', testY)
