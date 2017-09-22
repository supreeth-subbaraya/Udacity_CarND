import csv
import cv2
import numpy as np
import matplotlib.pyplot as plt

#measurements = []

#with open('data/driving_log.csv', 'r', newline='') as drivingLog:
  #reader = csv.reader(drivingLog, delimiter=',', skipinitialspace=True)
  #next(drivingLog, None)
  #for line in reader:
    #measurement = float(line[3])
    #measurements.append(measurement)
    #measurements.append(-measurement)
    
#plt.hist(measurements, bins=500)
#plt.show()

bgrImage = cv2.imread('data/IMG/center_2016_12_01_13_32_46_587.jpg')
rgbImage = cv2.cvtColor(bgrImage, cv2.COLOR_BGR2RGB);
croppedImage = rgbImage[50:137,:,:];
image = cv2.resize(croppedImage, (200,66));

flippedImage = cv2.flip(image, 1);
plt.imshow(croppedImage)
plt.show()
