import csv
import cv2
import numpy as np

images = []
measurements = []
correction = 0.2
corrections = [0, correction, -correction]

with open('data/driving_log.csv', 'r', newline='') as drivingLog:
  reader = csv.reader(drivingLog, delimiter=',', skipinitialspace=True)
  next(drivingLog, None)
  for line in reader:
    for i in range(3):
      imageFileName = 'data/' + line[i]
      bgrImage = cv2.imread(imageFileName)
      image = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2RGB);
      images.append(image)
      measurement = float(line[3]) + corrections[i]
      measurements.append(measurement)
    
    
X_train = np.array(images)
y_train = np.array(measurements)

print(X_train.shape)
print(y_train.shape)

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda
from keras.layers.convolutional import Conv2D
from keras.layers.pooling import MaxPooling2D

model = Sequential()
model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(160,320,3)))
model.add(Conv2D(filters = 6, kernel_size = (5,5), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2,2)))
model.add(Conv2D(filters = 16, kernel_size = (5,5), activation='relu'))
model.add(MaxPooling2D(pool_size=(2, 2), strides=(2,2)))
model.add(Flatten())
model.add(Dense(120, activation='relu'))
model.add(Dense(84, activation='relu'))
model.add(Dense(1))

model.compile(optimizer='adam', loss='mse')
model.fit(X_train, y_train, validation_split = 0.2, shuffle=True, epochs = 5)

model.save('model_LeNet.h5')