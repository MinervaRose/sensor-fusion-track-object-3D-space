3D Object Tracking  Final Project (FP.6)
Overview

The goal of this project is to track objects in 3D space using both camera and LiDAR data and to estimate the Time-to-Collision (TTC) between the ego vehicle and the car in front.

In this final part (FP.6), I compared the performance of several detector-descriptor combinations. To make this efficient, I updated the C++ code to accept command-line arguments and created a bash script to automatically test all valid combinations. The script ran the program for each pair, saved the results in CSV files, and summarized the results with a short Python script.

1. TTC from LiDAR

For the LiDAR part, I used the distance of points in front of the car to estimate TTC.
The process:

LiDAR points were cropped to focus on the ego lane.

Outliers were removed using the median of the nearest points.

TTC was then computed from the change in distance between frames.

Results (examples):

SIFT_BRISK LID Mean TTC = 11.7 s
FAST_BRISK LID Mean TTC = 11.7 s

Observation:
The LiDAR TTC values were very consistent (around 11-12 seconds). They are more stable because LiDAR measurements are not affected by lighting or keypoint matching errors.

2. TTC from Camera

For the camera part, I matched keypoints between consecutive frames and looked at how their distances changed. The change in these distances tells us how quickly the car in front is approaching.

Results (examples):

FAST_BRISK CAM Mean TTC = 13.3 s
SIFT_BRISK CAM Mean TTC = 11.5 s

Observation:
Camera-based TTC is more sensitive to the feature type.
SIFT and FAST worked very well with the BRISK descriptor, while ORB and SHITOMASI were less stable.

3. Detector-Descriptor Comparison

I tested multiple detector and descriptor combinations using my run_fp6.sh script.
Invalid combinations (like AKAZE descriptor with a non-AKAZE detector) were skipped automatically.

Detectors tested:

HARRIS, SHITOMASI, FAST, BRISK, ORB, AKAZE, SIFT

Descriptors tested:

BRISK, BRIEF, ORB, FREAK, AKAZE, SIFT

The Python script analyze_fp6.py generated a summary of all results.

Results Summary:(full table in fp6_summary.txt)
Combination	TTC Source	Frames	Mean TTC (s)	Median TTC (s)	StdDev	Comments
AKAZE_BRISK	CAM	18	12.65	12.53	2.62	Stable
AKAZE_BRISK	LID	18	11.73	12.25	2.37	Consistent
BRISK_BRISK	CAM	18	14.52	14.14	4.29	Slightly higher
FAST_BRISK	CAM	18	13.29	11.51	4.08	Good balance
SIFT_BRISK	CAM	18	11.50	11.75	2.66	Best match
ORB_BRISK	CAM	11	16.67	11.10	13.08	Noisy
SHITOMASI_BRISK	CAM	17	16.07	11.41	13.68	Less stable
(HARRIS_*)	CAM	0	nan	nan	nan	Failed to detect enough points

Observation:

SIFT + BRISK gave the most stable TTC values close to LiDAR.

FAST + BRISK was also a good, fast alternative.

ORB and SHITOMASI were less reliable for TTC estimation.

Harris combinations did not produce valid results.

4. Runtime and Stability
Detector	Speed	Stability
SHITOMASI	Fast	Moderate
FAST	Very fast	Good
BRISK	Fast	Good
ORB	Very fast	Variable
AKAZE	Slow	Stable
SIFT	Slowest	Most reliable

Binary descriptors (BRISK, ORB) are faster but noisier.
Gradient-based methods (SIFT, AKAZE) take longer but are more accurate and consistent.

5. Automation Setup

To make the testing process easier and reproducible:

The C++ code was modified to accept detector and descriptor names from command-line arguments.

A shell script (run_fp6.sh) runs all valid combinations automatically.

A Python script (analyze_fp6.py) reads the CSVs, filters invalid TTC values, and summarizes the results in a single table.

This made it possible to test all combinations in one go and quickly identify which performed best.

6. Reflections

This project taught me how much feature detector and descriptor choice affects TTC estimation.
LiDAR gives steady and reliable results, while camera TTC varies depending on how well keypoints are detected and matched.

Automating the testing saved a huge amount of time and helped compare all methods objectively.
SIFT with BRISK turned out to be the most balanced option between accuracy and runtime.

7. Conclusion

All valid detector-descriptor combinations were tested and analyzed.
LiDAR results remained stable across runs, and the best camera results were obtained with SIFT + BRISK and FAST + BRISK.
The entire workflow now runs automatically and produces clear, repeatable results.

Best Combination Overall: SIFT (detector) + BRISK (descriptor)
Fastest Reliable Combination: FAST + BRISK
Most Stable TTC (LiDAR): All consistent around 11.7 seconds











