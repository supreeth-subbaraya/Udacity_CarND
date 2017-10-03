
**Behavioral Cloning Project**

The goals / steps of this project are the following:
* Use the simulator to collect data of good driving behavior
* Build, a convolution neural network in Keras that predicts steering angles from images
* Train and validate the model with a training and validation set
* Test that the model successfully drives around track one without leaving the road
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./model_summary.png "Model Summary"
[image2]: ./model.png "Model"
[image3]: ./examples/placeholder_small.png "Recovery Image"
[image4]: ./examples/placeholder_small.png "Recovery Image"
[image5]: ./examples/placeholder_small.png "Recovery Image"
[image6]: ./examples/placeholder_small.png "Normal Image"
[image7]: ./examples/placeholder_small.png "Flipped Image"

## Rubric Points
---
### Files Submitted & Code Quality

#### 1. File Submitted

My project includes the following files:

* model.py containing the script to create and train the model
* drive.py for driving the car in autonomous mode
* model.h5 containing a trained convolution neural network 
* writeup_report.md summarizing the results
* video.mp4 showing the results

#### 2. Submission includes functional code
Using the Udacity provided simulator and my drive.py file, the car can be driven autonomously around the track by executing 
```sh
python drive.py model.h5
```

#### 3. Submission code is usable and readable

The model.py file contains the code for training and saving the convolution neural network. The file shows the pipeline I used for training and validating the model, and it contains comments to explain how the code works.

### Model Architecture and Training Strategy

#### 1. An appropriate model architecture has been employed

I have implemented the model given in the End to End Learning for Self-Driving Cars paper from NVIDIA. The model consists of the following layers. A normalization layer was used before the first layer (implemented using Lambda) and a Flatten layer was added b/w 5 and 6

|  Layer | Filters  | Kernel  | Stride  | Activation  |
|---|---|---|---|---|
|  1 | 24  | 5x5  |  2x2 |  RELU |
|  2 |  36 | 5x5  | 2x2  |  RELU |
|  3 |  48 | 5x5  | 2x2  |  RELU |
|  4 |  64 | 3x3  | 1x1  | RELU  |
|  5 |  64 | 3x3  | 1x1  |  RELU |

|  Layer | Neurons  |
|---|---|
|  6 | 1164  | 
|  7 |  100 |
|  8 |  50 |
|  9 |  10 |
|  10 |  1 |

#### 2. Attempts to reduce overfitting in the model

The model contains dropout layers with a dropout rate of 0.5

#### 3. Model parameter tuning

The model used an adam optimizer, so the learning rate was not tuned manually.

#### 4. Appropriate training data

The following is the training data used.

1. Left,Center, Right images of the given data from Udacity
2. Left,Center, Right images of the recovery data collected (recovery.zip)
3. Center images of anothr lap of the track (more_data.zip)

A correction of 0.2 was used for the left and right images.

### Architecture and Training

#### 1. Solution Design Approach

At the beginning I implemented the LeNet model in Keras and used the shipped dataset to train the network. I got good results just with this network, but there were a few places where the car went off track. 

Then I started to implement the NVIDIA model and trained it with just the center images. In order to improve I added the left and right images with correction factors. Data was split 80-20 % for training and validation. 

Then I looked at the data to see if there is a bias. I found that the data is biased towards left turn. Hence I flipped the images. Also since there is lot of irrelevant pixels in the image, i cropped it to [50:137] along the y-axis. The resized the image to (66,200) since the NVIDIA model's input size is (66,200)

This was not sufficient as the model could not recover from some situations. Hence i collected recovery data for this as suggested in the videos. Then I collected data for one more lap to give better results. By training with these datasets, the car was able to steer autonomously in the simulator

The epochs were chosen on trial and error method, i.e not allowing the validation losses to go high.

Also the drive.py was modified (images resized and cropped) to get it working with the model.

#### 2. Final Model Architecture

The model architecture has been discussed above. A summary of the model is as below

![alt text][image1]

The following is the plot of the model from keras.

![alt text][image2]

#### 3. Creation of the Training Set & Training Process

As mentioned above the following are the ways I collected data.

1. Drive for one lap, with smooth driving, driving mostly at the center
2. Recobery data: I would go towards the edges, start recording, recover and stop recording.
