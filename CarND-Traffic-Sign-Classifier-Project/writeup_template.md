# **Traffic Sign Recognition ** 

## Overview of the Project

This project builds a traffic sign classifier using the German Traffic Sign benchmark (GTRSB).

**Build a Traffic Sign Recognition Project**

The goals / steps of this project are the following:
* Load the data set (see below for links to the project data set)
* Explore, summarize and visualize the data set
* Design, train and test a model architecture
* Use the model to make predictions on new images
* Analyze the softmax probabilities of the new images
* Summarize the results with a written report


[//]: # (Image References)

[image1]: ./examples/visualization.jpg "Visualization"
[image2]: ./examples/grayscale.jpg "Grayscaling"
[image3]: ./examples/random_noise.jpg "Random Noise"
[image4]: ./examples/placeholder.png "Traffic Sign 1"
[image5]: ./examples/placeholder.png "Traffic Sign 2"
[image6]: ./examples/placeholder.png "Traffic Sign 3"
[image7]: ./examples/placeholder.png "Traffic Sign 4"
[image8]: ./examples/placeholder.png "Traffic Sign 5"

## Rubric Points

Here I will consider the [rubric points](https://review.udacity.com/#!/rubrics/481/view) individually and describe how I addressed each point in my implementation.  

### Writeup / README

The project is submitted to this github account and this is the writeup for the same. The files are Traffic_Sign_Classifier.ipynb and .html

### Data Set Summary & Exploration

* The size of training set is 34799
* The size of the validation set is 4410
* The size of test set is 12630
* The shape of a traffic sign image is (32, 32, 3)
* The number of unique classes/labels in the data set is 43

#### Exploratory visualization of the dataset.

Here is an exploratory visualization of the data set. It is a bar chart showing how the data is spread across the classes.

![alt text][image1]

### Design and Test a Model Architecture

#### Preprocessing

As a first step after viewing the training set, I preprocessed each of the images in training, test and validation test as follows.

1. Convert to grayscale
2. Equalize the histogram 
3. Normalize the image.

I tried using image augmentation to generate more training set. But it did not improve the accuracy which  might be due to the capacity of the network used


#### Model Architecture

I used the LeNet model with very few modifications.

My final model consisted of the following layers:

| Layer         		|     Description	        					| 
|:---------------------:|:---------------------------------------------:| 
| Input         		| 32x32x1 RGB image   							| 
| Convolution 5x5     	| 1x1 stride, 12 filters, same padding, outputs 28x28x12 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 14x14x32 				|
| Convolution 5x5	    | 1x1 stride, 32 filters, same padding, outputs 10x10x32 	|
| RELU					|												|
| Max pooling	      	| 2x2 stride,  outputs 5x5x32 				|
| Fully connected		| input 800, output 120        									|
| RELU					|												|
| Droput					|							keepProb 0.5					|
| Fully connected		| input 120, output 84        									|
| RELU					|												|
| Droput					|							keepProb 0.5					|
| Fully connected		| input 84, output 43        									|
| Softmax				|         									|
 


#### Train, Validate and Test the Model

My final model results were:

* Validation Set Accuracy 98.3
* Test set accuracy 95.2

The above mentioned modification to the LeNet model was used here. The following are some of the details.

1. Weight Initialization: Xavier initialization was used inspired from cs 231n class of Stanford. This is supposed to be a good weight initializer since it does not go to the extremes of weight initialization, i.e does not allowvalues to go very high or allow the weights to die out.

2. Learning rate: A exponential decay was used for the learning rate. Initial value for the learning rate was chosen between [1e-2 1e-3 1e-4] which was 1e-3

3. A regularization strength of 1e-4 was chosen to prevent overfitting using L2 regularization. This was also chosen using a similiar approach to the learning rate

4. EPOCH was changed to 100 and BATCH_SIZE was kept the same, 128.

5. Dropout layer: A droput probability of 0.5 was chosen since this value is used widely.

6. Adam optimizer was used for optimization

7. Increasing the depth of the conv layer improved the accuracies mostly due to the ability to detect different artifacts in the images.

### Test a Model on New Images

10 sign images were used which is present in the download_images/reshaped directory in this repo.


Here are the results of the prediction:

| Image			        |     Prediction	        					|  Certainity
|:---------------------:|:---------------------------------------------:|:---------------------:|
| 50-speed-limit      		| No passing for vehicles over 3.5 metric tons    									| 0.78 |
| Turn right ahead      			| Turn right ahead  										| 1 |
| Right-of-way at the next intersection					|  Right-of-way at the next intersection											| 1 |
| Road work      		| Road work				 				| 1 |
| Speed limit (70km/h)      		| Speed limit (70km/h) 					 				| 0.95 | 
| Speed limit (130km/h) 			| Speed limit (20km/h)       							| 0.43 | 
| Speed limit (100km/h) 			| No passing for vehicles over 3.5 metric tons      							| 0.82
| no-vehicles 			|Yield     							|
| Priority road 			| Priority road      							| 1 | 
| Slippery Road   | Right-of-way at the next intersection       							| 0.67 |


The model was able to guess 5/10 test images. One of the images was not in the dataset. Hence it would not be classified correctly. 
