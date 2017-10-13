
**Vehicle Detection Project**

The goals / steps of this project are the following:

* Perform a Histogram of Oriented Gradients (HOG) feature extraction on a labeled training set of images and train a classifier Linear SVM classifier
* Optionally, you can also apply a color transform and append binned color features, as well as histograms of color, to your HOG feature vector. 
* Note: for those first two steps don't forget to normalize your features and randomize a selection for training and testing.
* Implement a sliding-window technique and use your trained classifier to search for vehicles in images.
* Run your pipeline on a video stream (start with the test_video.mp4 and later implement on full project_video.mp4) and create a heat map of recurring detections frame by frame to reject outliers and follow detected vehicles.
* Estimate a bounding box for vehicles detected.

[//]: # (Image References)
[image1]: ./output_images/carImage.png
[image2]: ./output_images/notCarImage.png
[image3]: ./output_images/hogCarImage.png
[image4]: ./output_images/hogFeatures.png
[image5]: ./output_images/SpatialFeatures.png
[image6]: ./output_images/Histogram.png
[image7]: ./output_images/searchWindows.png
[image8]: ./output_images/searchWindowOut.png
[video1]: ./project_video.mp4

---
### Writeup / README

####1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Vehicle-Detection/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

The following is the contents of the writeup. My notebook in the repository is VehicelDetection.ipynb. 

### Histogram of Oriented Gradients (HOG)

#### 1. Explain how (and identify where in your code) you extracted HOG features from the training images.

The code for extracting the hog and color features is in the section "Extract Features" in the notebook

I started by reading in all the `vehicle` and `non-vehicle` images.  Here is an example of one of each of the `vehicle` and `non-vehicle` classes:

![alt text][image1]
![alt text][image2]

I explored some of the colorspaces and then settled on YCrCb since it was used in the lectures and also found the following result on one of the channel The following is the image and its hog feature on one of the channel.

![alt text][image3]
![alt text][image4]

The extract_features method also uses the spatial binning and color hisograms for the colorspace chosen. The following are the plots of these features on a car image.

![alt text][image5]
![alt text][image6]

####2. Explain how you settled on your final choice of HOG parameters.

The following were the parameters I used for hog features. Since these were used in the lecture class examples and worked well I used the following parameters. 

(orient, pix_per_cell, cell_per_block) = (9,8,2)

For color histogram I used 64 histogram bins and spatial_size of (32,32) for spatial binning. These were zeroed in on after trying out a coupe of other values for them.

####3. Describe how (and identify where in your code) you trained a classifier using your selected HOG features (and color features if you used them).

The section 'Train model' in notebook shows the training process for the classifier. The following was done

1. The hog, color histogram and spatial bin features were extracted using extract_festures
2. This was done for both the car and non-car images
3. These features combined together were normalized using StandardScaler()
4. Then this dataset was split into train-test with 80-20 breakout
5. LinearSVC() was used to train the classifier
6. A accuracy of 0.99 was ontained.

###Sliding Window Search

####1. Describe how (and identify where in your code) you implemented a sliding window search.  How did you decide what scales to search and how much to overlap windows?

The sliding window search is implemened in 'Search Windows' and 'Slide Window' sctions. The 'Find Cars' section implements this for the find_cars method.  The scales were chosen between 1 to 2.5 after checking with  test images and the video with the scales. The number of scales used was 10 in  this range. The following is the output of the search window.

![alt text][image7]

The following is the output after applying the find_cars method on the same image.

![alt text][image8]

####2. Show some examples of test images to demonstrate how your pipeline is working.  What did you do to optimize the performance of your classifier?

Ultimately I searched on two scales using YCrCb 3-channel HOG features plus spatially binned color and histograms of color in the feature vector, which provided a nice result.  Here are some example images:

![alt text][image4]
---

### Video Implementation

####1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (somewhat wobbly or unstable bounding boxes are ok as long as you are identifying the vehicles most of the time with minimal false positives.)
Here's a [link to my video result](./project_video.mp4)


####2. Describe how (and identify where in your code) you implemented some kind of filter for false positives and some method for combining overlapping bounding boxes.

I recorded the positions of positive detections in each frame of the video.  From the positive detections I created a heatmap and then thresholded that map to identify vehicle positions.  I then used `scipy.ndimage.measurements.label()` to identify individual blobs in the heatmap.  I then assumed each blob corresponded to a vehicle.  I constructed bounding boxes to cover the area of each blob detected.  

Here's an example result showing the heatmap from a series of frames of video, the result of `scipy.ndimage.measurements.label()` and the bounding boxes then overlaid on the last frame of video:

### Here are six frames and their corresponding heatmaps:

![alt text][image5]

### Here is the output of `scipy.ndimage.measurements.label()` on the integrated heatmap from all six frames:
![alt text][image6]

### Here the resulting bounding boxes are drawn onto the last frame in the series:
![alt text][image7]



---

###Discussion

####1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

