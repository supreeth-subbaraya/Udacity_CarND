import csv
import cv2
import numpy as np
import os

images = []
measurements = []
correction = 0.2
corrections = [0, correction, -correction]
dropoutProb = 0.5

# Load given data
with open('data/driving_log.csv', 'r', newline='') as drivingLog:
  reader = csv.reader(drivingLog, delimiter=',', skipinitialspace=True)
  next(drivingLog, None)
  for line in reader:
    for i in range(3):
    
      imageFileName = 'data/' + line[i]
      bgrImage = cv2.imread(imageFileName)
      rgbImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2RGB);
      croppedImage = rgbImage[50:137,:,:];
      image = cv2.resize(croppedImage, (200,66));
      images.append(image)
      measurement = float(line[3]) + corrections[i]
      measurements.append(measurement)
      
      flippedImage = cv2.flip(image, 1);
      images.append(flippedImage)
      measurements.append(-measurement)

# Load recovery data
with open('recovery_data/driving_log.csv', 'r', newline='') as drivingLog:
  reader = csv.reader(drivingLog, delimiter=',', skipinitialspace=True)
  next(drivingLog, None)
  for line in reader:
    for i in range(3):
    
      (dirName, filename) = os.path.split(line[i]);
      
      imageFileName = 'recovery_data/IMG/' + filename
      bgrImage = cv2.imread(imageFileName)
      rgbImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2RGB);
      croppedImage = rgbImage[50:137,:,:];
      image = cv2.resize(croppedImage, (200,66));
      images.append(image)
      measurement = float(line[3]) + corrections[i]
      measurements.append(measurement)

# Load more data
with open('more_data/driving_log.csv', 'r', newline='') as drivingLog:
  reader = csv.reader(drivingLog, delimiter=',', skipinitialspace=True)
  next(drivingLog, None)
  for line in reader:
    
    (dirName, filename) = os.path.split(line[0]);
    
    imageFileName = 'more_data/IMG/' + filename
    bgrImage = cv2.imread(imageFileName)
    rgbImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2RGB);
    croppedImage = rgbImage[50:137,:,:];
    image = cv2.resize(croppedImage, (200,66));
    images.append(image)
    measurement = float(line[3])
    measurements.append(measurement)

# Train

X_train = np.array(images)
y_train = np.array(measurements)

print(X_train.shape)
print(y_train.shape)

from keras.models import Sequential
from keras.layers import Flatten, Dense, Lambda, Dropout
from keras.layers.convolutional import Conv2D, Cropping2D

model = Sequential()
model.add(Lambda(lambda x: (x / 255.0) - 0.5, input_shape=(66,200,3)))
model.add(Conv2D(filters=24, kernel_size=(5,5), strides=(2,2), activation = 'relu'))
model.add(Conv2D(filters=36, kernel_size=(5,5), strides=(2,2), activation = 'relu'))
model.add(Conv2D(filters=48, kernel_size=(5,5), strides=(2,2), activation = 'relu'))
model.add(Conv2D(filters=64, kernel_size=(3,3), strides=(1,1), activation = 'relu'))
model.add(Conv2D(filters=64, kernel_size=(3,3), strides=(1,1), activation = 'relu'))
model.add(Flatten())
model.add(Dense(1164, activation='relu'))
model.add(Dropout(dropoutProb))
model.add(Dense(100, activation='relu'))
model.add(Dropout(dropoutProb))
model.add(Dense(50, activation='relu'))
model.add(Dense(10, activation='relu'))
model.add(Dense(1))

model.compile(optimizer='adam', loss='mse')
model.fit(X_train, y_train, validation_split = 0.2, shuffle=True, epochs = 5)

model.save('model.h5')
