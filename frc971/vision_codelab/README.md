# Vision Codelab
## Prerequisites: python knowledge, familiarity with pip and numpy (if not, google them and learn!)

In this codelab, you will be writing the vision code for the Galactic Search Challenge
(detailed on page 18 [here](https://firstfrc.blob.core.windows.net/frc2021/Manual/2021AtHomeChallengesManual.pdf)) from the 2021 at-home skills Challenges.


As explained in the manual, balls will be randomly placed in 4 possible configurations (alliance red/blue and field A/B).
From images of the field, you will be able to determine which path the balls are in so that the robot can drive with that respective spline.

If you are doing this locally and not on the build server, make sure you have python3 and install the needed packages with
`pip3 install numpy matplotlib opencv-python absl-py glog`

Then, read/watch these opencv tutorials in order, which you will need for the codelab:
- [Intro](https://www.geeksforgeeks.org/introduction-to-opencv/)
- [Color spaces](https://www.geeksforgeeks.org/color-spaces-in-opencv-python/?ref=lbp)
- [Converting between color spaces](https://www.geeksforgeeks.org/python-opencv-cv2-cvtcolor-method/?ref=gcse)
- MOST IMPORTANT: [Filtering colors](https://pythonprogramming.net/color-filter-python-opencv-tutorial/)
- [Erosion and dilation](https://www.geeksforgeeks.org/erosion-dilation-images-using-opencv-python/)

Now, follow the TODOs in codelab.py, and run ./codelab_test.py to test your code.
