# SFND 3D Object Tracking

Welcome to the final project of the camera course. By completing all the lessons, you now have a solid understanding of keypoint detectors, descriptors, and methods to match them between successive images. Also, you know how to detect objects in an image using the YOLO deep-learning framework. And finally, you know how to associate regions in a camera image with Lidar points in 3D space. Let's take a look at our program schematic to see what we already have accomplished and what's still missing.

<img src="images/course_code_structure.png" width="779" height="414" />

In this final project, you will implement the missing parts in the schematic. To do this, you will complete four major tasks: 
1. First, you will develop a way to match 3D objects over time by using keypoint correspondences. 
2. Second, you will compute the TTC based on Lidar measurements. 
3. You will then proceed to do the same using the camera, which requires to first associate keypoint matches to regions of interest and then to compute the TTC based on those matches. 
4. And lastly, you will conduct various tests with the framework. Your goal is to identify the most suitable detector/descriptor combination for TTC estimation and also to search for problems that can lead to faulty measurements by the camera or Lidar sensor. In the last course of this Nanodegree, you will learn about the Kalman filter, which is a great way to combine the two independent TTC measurements into an improved version which is much more reliable than a single sensor alone can be. But before we think about such things, let us focus on your final project in the camera course. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* Git LFS
  * Weight files are handled using [LFS](https://git-lfs.github.com/)
  * For Linux:
    * Install using `sudo apt install git-lfs`
    * Once the repo directory has been cloned, run `git lfs pull` to pull large files in.
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level project directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./3D_object_tracking`.

## Projcect implementation

### FP.0 Final Report

Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf. The writeup / README should include a statement and supporting figures / images that explain how each rubric item was addressed, and specifically where in the code each step was handled. This README file will detail how I solved this project.

### FP.1 Match 3D Objects

Implement the method "matchBoundingBoxes", which takes as input both the previous and the current data frames and provides as output the ids of the matched regions of interest (i.e. the boxID property). Matches must be the ones with the highest number of keypoint correspondences.

I completed this by filling out `matchBoundingBoxes(...)`. In this method I looped through the list of matches and checked if they were in both frames. If they were I stored the value of that match in a matrix and incremented the match value.

Finally, I counted the occurances in that matrix and selected the one with more keypoints tallied.

### FP.2 Compute Lidar-based TTC

Compute the time-to-collision in second for all matched 3D objects using only Lidar measurements from the matched bounding boxes between current and previous frame.

This was completed in the `computeTTCLidar(...)` method. In this method I average the x values of the previous and current frame ensuring that they were within the lane by checking the y values of each point. I then used the difference in the x values divided by  1 / frameRate to find the current speed. I then calculated TTC by divided the current frame's x value by the calculated speed.

### FP.3 Associate Keypoint Correspondences with Bounding Boxes

Prepare the TTC computation based on camera measurements by associating keypoint correspondences to the bounding boxes which enclose them. All matches which satisfy this condition must be added to a vector in the respective bounding box.

This was implemented in the `clusterKptMatchesWithROI` method. This method checks to see if the bounding box contains the keypoint. If so, it is pushed into the bounding box's list of keypoint matches.

### FP.4 Compute Camera-based TTC

Compute the time-to-collision in second for all matched 3D objects using only keypoint correspondences from the matched bounding boxes between current and previous frame.

This was implemented in the `computeTTCCamera` method. This method follows the example exercise from Lesson 3: Engineering a Collision Detection System, [Concept 3: Estimating TTC with a Camera](https://classroom.udacity.com/nanodegrees/nd313/parts/1971021c-523b-414c-93a3-2c6297cf4771/modules/3eb3ecc3-b73d-43bb-b565-dcdd5d7a2635/lessons/dfe71db5-4233-4e4f-b33f-40cb9899dc13/concepts/daceaff3-1519-4f4c-82ff-16e02b5c2e8f).

### FP.5 Performance Evaluation 1

Find examples where the TTC estimate of the Lidar sensor does not seem plausible. Describe your observations and provide a sound argumentation why you think this happened.

Generally, it seems like the Lidar provided a very good estimate of the time-to-collision. Ostensibly, this is because lidar can provide a very precise distance measurement. If you can filter the point cloud data to only concentrate on a specific target, this can be really effecty for getting the distance to the next car. Framerate tends to be fairly consistent, so it becomes realtively straight forward to estimate relative velocity and the time to collision.

### FP.6 Performance Evaluation 2

Run several detector / descriptor combinations and look at the differences in TTC estimation. Find out which methods perform best and also include several examples where camera-based TTC estimation is way off. As with Lidar, describe your observations again and also look into potential reasons.

Generally, the camera based method is not very good. Often, it yields negative times to collision which isn't useful. When this method does yield sensible and accurate results is when the distance ratio is greater than one.