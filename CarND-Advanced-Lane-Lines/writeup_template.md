## Writeup Template

**Advanced Lane Finding Project**

The goals / steps of this project are the following:

* Compute the camera calibration matrix and distortion coefficients given a set of chessboard images.
* Apply a distortion correction to raw images.
* Use color transforms, gradients, etc., to create a thresholded binary image.
* Apply a perspective transform to rectify binary image ("birds-eye view").
* Detect lane pixels and fit to find the lane boundary.
* Determine the curvature of the lane and vehicle position with respect to center.
* Warp the detected lane boundaries back onto the original image.
* Output visual display of the lane boundaries and numerical estimation of lane curvature and vehicle position.

[//]: # (Image References)

[image1]: ./output_images/findchessBoard_Input.jpg "Input"
[image2]: ./output_images/findChessBoard_Output.png "Output"
[image3]: ./output_images/undistorted_input.jpg "Input"
[image4]: ./output_images/undistorted_output.png "undistorted output"
[image5]: ./output_images/undistorted_test_output.png "undistorted test output"
[image6]: ./output_images/straight_lines1.jpg "color thresh input"
[image7]: ./output_images/color_thresholded.png "color thresh"
[image8]: ./output_images/transformed.png "transformed"
[image9]: ./output_images/linePixels.png "line pixels"
[image10]: ./output_images/output.png "output"

[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

This is the readme file for the project. In the file any reference to notebook or code is referred to the python notebook in the repository, 'Advanced_Lane-line.ipynb'

### Camera Calibration

#### 1. Briefly state how you computed the camera matrix and distortion coefficients. Provide an example of a distortion corrected calibration image.
The camera calibration is implemented in the calbrateCamera() method of the uploaded jupyter notebook in the repo.

1. The pattern size of the provided images is (9,6)
2. Based on this size, I create object points which are locations of the form (x,y,z)
3. Then I find the checkerboard points for each of the image. The following is an example on an image.

![alt text][image1] ![alt text][image2]

4. After having the object points and image points from all the given images, I use OpenCV's calibrateCamera() to get the camera matrix, distortion coefficients, rotation and translation vectors.

5. Then I use the undistort image from opencv to obtain the following output.

![alt text][image3]
![alt text][image4]

### Pipeline (single images)

#### 1. Provide an example of a distortion-corrected image.

I applied the above mentioned undistort technique to one of the test images to obtain the following undistorted image.

![alt text][image5]

#### 2. Describe how (and identify where in your code) you used color transforms, gradients or other methods to create a thresholded binary image.  Provide an example of a binary image result.

Looking at the images and the video, it seemed that the pipeline would give good results if only the yellow and white line were captured. Hence i tried to use different thresholds for each of the color. I used two formats of the test images here, one HLS image and other the normal RGB one. I noted down the values of yellow and white lines in each of the hls and rgb images for each of the test image. Once obtained it seemed the colors lied in the following buckets

| Color         | Type          |      Max       |        Min        |
|:-------------:|:-------------:| :-------------:| :-------------:| 
| Yellow        | HLS           |   (15,100,180) | (25,165,255) |
| White        | RGB           |   (200,200,200) | (255,255,255) |

After obtaining this I applied these thresholds and implemented the colorThreshold() method in the code to obtain the following output.

![alt text][image6]
![alt text][image7]

#### 3. Describe how (and identify where in your code) you performed a perspective transform and provide an example of a transformed image.

The perspective transform is implemented in the jupyter notebook by the method transformImage(). I pass the source and destination points for the transformation.

```python
srcPoints = np.float32([ [263,imageSize[0]], [560,470], [720,470], [1046,imageSize[0]] ])
dstPoints = np.float32([ [100,imageSize[0]], [100,0], [1000,0], [1000,imageSize[0]] ])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 263, 720      | 100, 720        | 
| 560, 470      | 100, 0      |
| 720, 470     | 1000, 0      |
| 1046, 720      | 1000, 720        |

The following is an example of the warped image which shows how the parallel lines are preserved

![alt text][image8]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

The following is the pipeline for identifying the lane line pixels. This is implemented from lines 27-138 in 'Pipeline' section of the notebook.

1.  After obtaining the perspective transform of the binary thresholded image, we find the histogram column wise.
2. We do a sliding window approach as given in the class lectures on the two peaks of the histogram to obtain the x,y points of the left and right curves.
3. We then obtain the left and right fit using the polyfit() method. This is done in the pipeline() method in the notebook.
4. Once the line is found then we can just constrain our window of search in the subsequent frames as suggested in the lectures

The following is an image with the line fitted on the lane pixels.

![alt text][image9]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

The radius of curvature and posiion is calculated in the section 'Pipeline' in lines 138-160. I used the same method mentioned in the lectures. 

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

The following is the result image.

![alt text][image10]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./output_images/project_video_output.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

1. Initially the pipeline was not working fine since the binary thresholded image included all the pixels and was thresholded on only the S-channel. Understanding that a good threshold would result in good results tool some time.
2. The pipeline is likely to fail in the event of another vehicle with a white or yellow color in the front.
3. Currently no averaging is done on some previous n number of lane lines. Hence at some places it is jittery. This can be avoided by averaging
