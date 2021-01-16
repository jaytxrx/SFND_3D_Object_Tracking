## Final Project Report - Track an Object in 3D Space

This report describes how the assignments points were addressed.

Below shown is the building block diagram
<img src="images/course_code_structure.png" width="779" height="414" />

In the midterm project we computed the keypoints, descriptors and their matches for the entire image and not for the preceeding car alone.
In the lesson 6 (Combining Lidar and Camera), the lidar points were selected based on the bounding box created by Neural Network Yolo framework and projected on to the image.

In the final project, the keypoint descriptors obtained from image processing will be selected based on the Yolo bounding boxes for cars. This is done as we are only interested in the points on the cars. Any other points on other objects will cause error/confusion to the collision avoidance system. After that time to collision will be computed.

### FP.1 Match 3D Objects

<img src="report-files/BoundingBox.gif" />

In this section, step 8 *Track 3D object bounding boxes* needs to be implemented.
We need to associate bounding boxes between neighbouring frames by looking at the keypoint correspondences within those boundding boxes.

As the name of the function says, *matchBoundingBoxes()* matches the bounding box found in previous image to the current image. This is something like tracking bounding boxes. It can happen that in the first image, the bouding box might have id1 but in the second image, that bouding box might be labeled id2 (id1 might be pointing to another vehicle which we shouldn't use).

In the function *matchBoundingBoxes()*, parameter `matches` has the matched keypoints between previous and current frame. Frame details can be obtained from paremeters `prevFrame` and `currFrame`. The same keypoints included in bounding box in `prevFrame` and `currFrame` needs to be found and matched. This matching is updated inside `bbBestMatches`.

The `struct DataFrame` contains all the inputs and outputs needed for the step 8. `DataFrame` has the `BoundingBox` struct which has the information about the list of all the bounding boxes in the current image in `DataFrame`. 

Note: Normally `trainIdx` should have been referencing the *previous image* as the keypoints and its descriptors are trained on previous image. Also `queryIdx` should have been referencing the *current image*. But it seems the authors have swapped these ID references by mixing the order in `matchDescriptors()`

Solution logic is as follows
* LOOP through each of the match points pair
* In the current match point `DMatch`, access the previous image keypoint`queryIdx` and current image keypoint `trainIdx`
* LOOP through all the previous image bounding boxes
* IF any of the previous image bounding boxes contains the previous image keypoint (ELSE go to match point LOOP)
* THEN LOOP through all the current image bounding boxes
* IF any of the current image bounding boxes contains the current image keypoint  (ELSE go to previous image bounding boxes LOOP)
* THEN store the previous and current box id match and increment the counter for that box pair
* After the match point pair LOOP is completed, LOOP through the box pairs to find the largest matches in each pairs


### FP.2 : Compute Lidar-based TTC

To filter out sudden jumps in the lidar values, average distance reported by the lidar points are used for computation. Also only lidar points in our lane is used for computation (others are neglected). TTC is computed using the formula *TTC = avgCurrDist * dT / (avgPrevDist - avgCurrDist);*

### FP.3 : Associate Keypoint Correspondences with Bounding Boxes

In this section we need to find if a keypoint is inside the provided bounding box(or ROI) - this is done using the `contains` function. Then for all the that keypoint in ROI we take the mean value of the distance  (which can be accessed from `DMatch.distance`). Any distance value which is far away (more than 70%) is neglected.

### FP.4 : Compute Camera-based TTC

<img src="report-files/TTC.gif" />

To compute the distance from camera images, distance between each of the keypoint is computed form the current and the previous frame. Their distance ratio can used to calculate the TTC. As there are multiple keypoints, there will be multiple distance ration. Median of the distance ratio is used to reduce the error caused by outliers. At the end TTC is computed with the median distance ratio using the formula `TTC = -dT / (1 - medDistRatio);`

### FP.5 : Performance Evaluation 1

While visualising the top view of the lidar points, it was not possible to see many outliers. All lidar points were close by. Also tried mean and median based TTC calculation and was not able to find major differences.

### FP.6 : Performance Evaluation 2

Below table was created using the following steps
1) For each entries in [CSV file](report-files/report.csv), the difference between *TTC Lidar* and *TTC Camera* was calculated.
Note that *NAN* and *-inf* rows were deleted
2) The sum of the differences for detector/descriptor is updated in *Diff* column
3) *Diff* column was arranged in ascending order

Based on the below table, ***AZAKE/BRISK*** is the best detector descriptor combination

| Detector Type | Descriptor Type | Diff    |
|---------------|-----------------|---------|
| AKAZE         | BRISK           | 16.516  |
| AKAZE         | BRIEF           | 16.0164 |
| AKAZE         | SIFT            | 16.9908 |
| AKAZE         | FREAK           | 17.0215 |
| AKAZE         | ORB             | 16.4344 |
| SHITOMASI     | FREAK           | 16.6747 |
| SIFT          | SIFT            | 16.3586 |
| HARRIS        | SIFT            | 15.85   |
| SIFT          | BRISK           | 16.644  |
| SHITOMASI     | BRIEF           | 15.6108 |
| SHITOMASI     | SIFT            | 14.8138 |
| SHITOMASI     | ORB             | 14.4815 |
| SHITOMASI     | BRISK           | 14.313  |
| SIFT          | BRIEF           | 13.3283 |
| SIFT          | FREAK           | 11.7047 |
| HARRIS        | FREAK           | 9.5189  |
| FAST          | SIFT            | 9.0764  |
| HARRIS        | BRIEF           | 9.0114  |
| ORB           | FREAK           | 8.6231  |
| HARRIS        | ORB             | 8.5131  |
| FAST          | FREAK           | 7.524   |
| HARRIS        | BRISK           | 7.165   |
| FAST          | BRIEF           | 6.9426  |
| FAST          | ORB             | 4.7172  |
| FAST          | BRISK           | 3.7157  |
| BRISK         | BRIEF           | 2.1931  |
| BRISK         | FREAK           | 0.8253  |


#### References
* [Image to GIF conversion](https://askubuntu.com/questions/648244/how-do-i-create-an-animated-gif-from-still-images-preferably-with-the-command-l#:~:text=From%20GIMP%20go%20to%20File,to%20the%20GIF%20export%20options.)
* [CSV to markdown website](https://thisdavej.com/copy-table-in-excel-and-paste-as-a-markdown-table/)
