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
[video1]: ./project_video.mp4 "Video"

## [Rubric](https://review.udacity.com/#!/rubrics/571/view) Points

### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  [Here](https://github.com/udacity/CarND-Advanced-Lane-Lines/blob/master/writeup_template.md) is a template writeup for this project you can use as a guide and a starting point.  

This is the readme file for the project

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

The code for my perspective transform includes a function called `warper()`, which appears in lines 1 through 8 in the file `example.py` (output_images/examples/example.py) (or, for example, in the 3rd code cell of the IPython notebook).  The `warper()` function takes as inputs an image (`img`), as well as source (`src`) and destination (`dst`) points.  I chose the hardcode the source and destination points in the following manner:

```python
src = np.float32(
    [[(img_size[0] / 2) - 55, img_size[1] / 2 + 100],
    [((img_size[0] / 6) - 10), img_size[1]],
    [(img_size[0] * 5 / 6) + 60, img_size[1]],
    [(img_size[0] / 2 + 55), img_size[1] / 2 + 100]])
dst = np.float32(
    [[(img_size[0] / 4), 0],
    [(img_size[0] / 4), img_size[1]],
    [(img_size[0] * 3 / 4), img_size[1]],
    [(img_size[0] * 3 / 4), 0]])
```

This resulted in the following source and destination points:

| Source        | Destination   | 
|:-------------:|:-------------:| 
| 585, 460      | 320, 0        | 
| 203, 720      | 320, 720      |
| 1127, 720     | 960, 720      |
| 695, 460      | 960, 0        |

I verified that my perspective transform was working as expected by drawing the `src` and `dst` points onto a test image and its warped counterpart to verify that the lines appear parallel in the warped image.

![alt text][image4]

#### 4. Describe how (and identify where in your code) you identified lane-line pixels and fit their positions with a polynomial?

Then I did some other stuff and fit my lane lines with a 2nd order polynomial kinda like this:

![alt text][image5]

#### 5. Describe how (and identify where in your code) you calculated the radius of curvature of the lane and the position of the vehicle with respect to center.

I did this in lines # through # in my code in `my_other_file.py`

#### 6. Provide an example image of your result plotted back down onto the road such that the lane area is identified clearly.

I implemented this step in lines # through # in my code in `yet_another_file.py` in the function `map_lane()`.  Here is an example of my result on a test image:

![alt text][image6]

---

### Pipeline (video)

#### 1. Provide a link to your final video output.  Your pipeline should perform reasonably well on the entire project video (wobbly lines are ok but no catastrophic failures that would cause the car to drive off the road!).

Here's a [link to my video result](./project_video.mp4)

---

### Discussion

#### 1. Briefly discuss any problems / issues you faced in your implementation of this project.  Where will your pipeline likely fail?  What could you do to make it more robust?

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  
