import os
from numpy.core.fromnumeric import shape
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import pandas as pd
import numpy as np
import re
import random

from keras.models import Sequential
from keras.layers import Dense
from keras import optimizers
# from keras.utils import to_categorical
from tensorflow.keras.utils import to_categorical


data_split_tuner_value =10  #13 for succ #total 50
epoch_number = 200
batch_number = 16
validation_ratio = 0.2


''' Main Code '''
# Clear the dataset from while lines and text, save the last acquisition number
format = re.compile('\d+\,[0-9]{1}\,-?(\d+)\,-?(\d+)\,-?(\d+)')
# format = re.compile('\d+\,[A-Z]{1}\,-?(\d+)\,-?(\d+)\,-?(\d+)')
# print(re.search(format,'1,a,123,-15,-2566').group()) example on data format

log_file_path = "dataset/data_motion_recognition/keras2.3/boxing_imp.txt" #data_log_temp.txt first_succ_data/data_log.txt
dataset_path = "dataset/data_motion_recognition/keras2.3/dataset.txt"
with open(log_file_path,'r') as data_file:
    with open(dataset_path,'w') as out_file:
        for counter,line in enumerate(data_file):
            if re.search(format,line):
                out_file.write(line)

# Now use panda to handle the dataset
columnNames = ['acquisition','activity','ax','ay','az','temperature', 'humidity', 'pressure'] # add activity instead letter ,'temperature'
dataset = pd.read_csv(dataset_path,header = None, names=columnNames,na_values=',')
# # find the number
last_index = max(np.unique(dataset.acquisition))

second_axis = []
for acq_index in range(1,last_index):
    second_axis.append(dataset[dataset.acquisition == acq_index].shape[0])


# # print(min(second_axis))

dtensor = np.empty((0,3*min(second_axis))) #change shape of dtensor
labels= np.empty((0))

for acq_index in range(2,last_index):
    temp = dataset[dataset.acquisition == acq_index]
    ax = temp.ax
    ay = temp.ay
    az = temp.az
    temperature = temp.temperature
    #humidity = temp.humidity
    #pressure = temp.pressure
    dtensor = np.vstack([dtensor,np.concatenate((ax, ay, az))])#, temperature, humidity, pressure))]) #focus on temp as well
    labels = np.append(labels,np.unique(temp.activity))


labels = np.asarray(pd.get_dummies(labels),dtype = np.int8)

#print(labels.shape)

sample_index = np.arange(0,dtensor.shape[0])
shuffled_indexes = np.random.shuffle(sample_index)
#shuffled_indexes = random.sample(list(sample_index), len(list(sample_index)))
#print(shuffled_indexes)


train_data = dtensor[sample_index[data_split_tuner_value:],:] #it was 20 i made it 6
test_data = dtensor[sample_index[:data_split_tuner_value],:]
train_labels = labels[sample_index[data_split_tuner_value:],:]
test_labels = labels[sample_index[:data_split_tuner_value],:]





train_shape = train_data.shape[1]
# print(train_shape)
# print(train_data.shape)
#print(test_data.dtype)
# print(train_labels.shape)
#print(test_labels.dtype)

model = Sequential()
model.add(Dense(128,input_shape =(train_shape,),name='input_layer'))
model.add(Dense(64, activation = 'relu', name='hidden1'))
#model.add(Dense(64, activation = 'relu', name='hidden2')) #another hidden layer 128
model.add(Dense(32, activation = 'relu', name='hidden3')) #another hidden layer
#model.add(Dense(32, activation = 'relu', name='hidden4')) #another hidden layer
model.add(Dense(4, activation='softmax' , name = 'output_layer')) #softmax #dont forget to put output number
#with softmax it returns 4 probability values that sums to one
#4 activities are: up and down (UND), Round Clock(ROU), Round counter Clock (ARO), triangular(TRG), 1>UND, 2>ROU, 3>ARO, 4>TRG
model.compile(optimizer= 'rmsprop', loss='categorical_crossentropy', metrics=['accuracy']) #use sparse is each letter is an integer (es a->1 b->2 c->3 ..) #it was rmsprop now adam
model.summary()

model.fit(train_data,train_labels,epochs=epoch_number, batch_size=batch_number, validation_split=validation_ratio , verbose=1)
results = model.evaluate(test_data, test_labels, verbose=1)

results_names = model.metrics_names
result = "\nThe %s value is: %f \nThe %s value is: %f \n" %(results_names[0] ,results[0],results_names[1] ,results[1])
print(result)
print(f"""Ratio trainded data  {len(train_data)/len(sample_index)}
          Ratio tested data {len(test_data)/ len(sample_index)}""")



model.save('trained_model/motion_recognition/keras2.3/boxing.h5')
f = open("trained_model/motion_recognition/keras2.3/boxing_log.txt", "w")
f.write(result)
f.close()
np.save('trained_model/motion_recognition/keras2.3/validation_data_boxing.npy', test_data)
np.save('trained_model/motion_recognition/keras2.3/validation_labels_boxing.npy', test_labels)

# # model = Sequential()
# # model.add(Dense(32,input_shape =(train_shape,),name='input_layer'))
# # model.add(Dense(64, activation = 'relu', name='hidden1'))
# # model.add(Dense(64, activation = 'relu', name='hidden2')) #another hidden layer 128
# # #model.add(Dense(32, activation = 'relu', name='hidden3')) #another hidden layer
# # #model.add(Dense(32, activation = 'relu', name='hidden4')) #another hidden layer
# # model.add(Dense(4, activation='softmax' , name = 'output_layer')) #softmax #dont forget to put output number
# # #with softmax it returns 4 probability values that sums to one
# # #4 activities are: up and down (UND), Round Clock(ROU), Round counter Clock (ARO), triangular(TRG), 1>UND, 2>ROU, 3>ARO, 4>TRG
# # model.compile(optimizer= 'rmsprop', loss='categorical_crossentropy', metrics=['accuracy']) #use sparse is each letter is an integer (es a->1 b->2 c->3 ..) #it was rmsprop now adam
# # model.summary()