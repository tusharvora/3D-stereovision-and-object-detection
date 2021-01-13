# 3D-stereovision-and-object-detection
The project report and the video can be found here- https://tusharvora.weebly.com/3d-machine-vision.html
- The repository contain code files written in MATLAB.
# Project Overview
-This project was part of  a course "3d machine vision" in Carleton University tauhgt by Professor Paul Harrison.
-The project uses stereo machine vision concepts to autonomously identify objects ,pose estimation and the real world depth.
-The program was written on MATLAB software to complete this task without using MATLAB inbuilt image processing tools.

# Problem identification / Challenge
-To identify the objects like hammer and screw driver automatically based on the stereo images provided for space robot application.
-To calculate the 3d depth of the objects.
-To estimate pose of the objects

# Process / Solution
The Matlab code is developed to complete the task which contains comments for self explanation.The main file name is "Final_3d_code.m"
-Camera calibration was performed manually by selecting similar points on both the stereo images.
-Object detection and object identification was performed by blob analysis technique.
-The points in one stereo image was matched on the another image by local stereo matching technique i.e Sum of square difference of intensities
-The rectified points were matched along the epipolar line of the respective points in second image 
-The depth of the object points was identified using stereo 3d reconstruction technique and
-The pose and orientation of the tools were identified using vector analysis.
-The detailed report can be seen below for more clarification.
 
# Result
-Successful in automatic process within accuracy of around 95% as required in the projects
-The time require was significantly less (from several minutes to only around 20 seconds ) because of some change in strategies.
