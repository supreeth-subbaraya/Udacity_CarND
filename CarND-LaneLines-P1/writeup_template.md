# **Finding Lane Lines on the Road** 

## Overview of the Project

The goal of the project is to develop a pipeline to detect and plot the lane lines given an image or a video. In this project we use some of the tools in the Computer Vision domain to develop this pipeline


[//]: # (Image References)

[image1]: ./test_images-output/hsvImg.png "HSV"

[image2]: ./test_images-output/maskedImage.png "Masked image"

[image3]: ./test_images-output/edgeImage.png "Canny edge detection output"

[image4]: ./test_images-output/roiImage.png "ROI for further processing"

[image5]: ./test_images-output/houghImage.png "Hough lines"

[image6]: ./test_images-output/combinedImage.png "Final result"

---

### Reflection

### 1. Describe your pipeline. As part of the description, explain how you modified the draw_lines() function.

In this project the following pipeline is used to detect and plot the lane lines

1. Convert the image from RGB color space to HSV color space. This is done to have minimal impact from the lighting conditions, timing of the day etc.

![alt text][image1]

2. Gaussian filter with a kernel size of 3 is applied to this image to smoothen it.

3. A mask is generated using HSV color thresholds for yellow and white lines. The mask is applied to the image.

![alt text][image2]

4. The image is then converted to grayscale image for further processing.

5. Canny edge detector is applied to the image with thresholds of [50,150] to detect the edges in the masked image.

![alt text][image3]

6. In order to avoid noise due to other features in the image, a Region of interest (ROI) is selected. This will focus the    view on ego lane.

![alt text][image4]

7. Then houghlines is used to detect the lines and further processing is done as explained below to obtain both the left      and right lane lines.

![alt text][image5]

8. Once obtained the lane lines are drawn on an image.

![alt text][image6]

In this project the helper functions given in the project is used to perform most of the above mentioned processing.

Modifications to the draw_lines() method:

1. The lines were obtained from the hough_lines method as an input

2. The slopes of these lines were calculated to classify the lines between two classes, left or right. In order to do this     a threshold of 0.2 was used. ( slope < -0.2 => right and slope > 0.2 => left )

3. Then the difference between the slope and the slope of the respective lane line draw in the previous image was compared    to see if it is within a tolerance level. This way we filter some outliers.

4. After obtaining the lane lines "polyfit" method from numpy was used to obtain the co-efficients of a line.

5. In order to draw a line the following points were used: (x1, imageHeight), (x2, 0.65 * imageHeight). z1 and x2 were        calculated using the co-efficients of the lines

6. Slopes of both the lines are stored for use in the upcoming frames.


### 2. Identify potential shortcomings with your current pipeline

The following are some of the shortcomings of the pipeline.

1. The pipeline still might not work in all lighting conditions or color of the road. This is noticeable in  the challenge    video where at a point the left lane goes off.

2. The pipeline might not work very well on curved roads since it uses a linear model to fit the line


### 3. Suggest possible improvements to your pipeline

1. Possible improvement is to further remove outliers using some kind of averaging with some n number of frames

2. Also, another improvement would be to improve the detection. Hough lines detects a lot of outliers. 

3. The final one is to use a better model for lane lines.
