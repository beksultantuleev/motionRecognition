import os
os.environ['TF_CPP_MIN_LOG_LEVEL'] = '2'

import pandas as pd
import numpy as np
import re

from keras.models import Sequential
from keras.layers import Dense, LSTM, GRU, Bidirectional
from keras import optimizers
from keras.utils import to_categorical


''' Main Code '''
# Clear the dataset from while lines and text, save the last acquisition number
format = re.compile('\d+\,[AEIOU]\,-?(\d+)\,-?(\d+)\,-?(\d+)')
# print(re.search(format,'1,a,123,-15,-2566').group()) example on data format

log_file_path = "dataset/log_file.txt"
dataset_path = "dataset/dataset.txt"
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

second_axis = []
for acq_index in range(2,last_index):
    second_axis.append(dataset[dataset.acquisition == acq_index].shape[0])
n_data = min(second_axis)
print(n_data)

#Tensor initialization
dtensor = np.empty((0,n_data,3))
labels= np.empty((0))

#SOLVE THA FACT THE VALUE 1 IS NOT TAKEN
for acq_index in range(1,last_index):
    temp = dataset[dataset.acquisition == acq_index]
    ax = temp.ax
    ay = temp.ay
    az = temp.az
    dtensor = np.vstack([dtensor,np.dstack((ax, ay, az))])
    labels = np.append(labels,np.unique(temp.letter))

print(dtensor.shape)
labels = np.asarray(pd.get_dummies(labels),dtype = np.int8)
#print(labels.shape)

sample_index = range(0,dtensor.shape[0])
shuffled_indexes = np.random.shuffle(sample_index)

train_data = dtensor[sample_index[20:],:]
test_data = dtensor[sample_index[:20],:]
train_labels = labels[sample_index[20:],:]
test_labels = labels[sample_index[:20],:]

print(train_data.shape)
print(test_data.dtype)

model = Sequential()
#model.add(LSTM(128, input_shape =(n_data,3), name='LSTM'))
model.add(GRU(128, input_shape =(n_data,3), name='LSTM'))
model.add(Dense(5, activation='softmax' , name = 'output_layer'))

model.compile(optimizer= 'rmsprop', loss='categorical_crossentropy', metrics=['accuracy']) #use sparse is each letter is an integer (es a->1 b->2 c->3 ..)
model.summary()

model.fit(train_data,train_labels, epochs=15, batch_size=10, validation_split=0.2 , verbose=2)
results = model.evaluate(test_data, test_labels)

results_names = model.metrics_names
result = "\nThe %s value is: %f \nThe %s value is: %f \n" %(results_names[0] ,results[0],results_names[1] ,results[1])
print(result)

model.save('./rnn_net.h5')
open( "./training_info.txt", 'wb').write(result)
np.save('./test_data_rnn.npy', test_data)
np.save('./test_labels_rnn.npy', test_labels)
np.save('./train_data_rnn.npy', train_data)
np.save('./test_labels_rnn.npy', test_labels)
