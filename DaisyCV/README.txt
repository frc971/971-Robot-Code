http://www.chiefdelphi.com/media/papers/2676?

Team 341 Vision System Code
By: Jared341
New: 04-30-2012 12:55 PM
Updated: 09-24-2012 07:05 PM

SmartDashboard Widget used by FIRST Team 341 for automatic targeting during the 2012 competition season.

Now that the 2012 FIRST Championships are a (painful) memory, Team 341 is releasing its laptop-based vision system code to all teams so that they can learn from it and/or use it in offseason competitions.

Attached Files
--------------
DaisyCV Team 341 Vision Tracking Widget
DaisyCV.zip
download file uploaded: 04-30-2012 12:55 PM
filetype: zip
filesize: 82.63kb
downloads: 1096

Sample Images
Team341TestImages.zip
download file uploaded: 09-24-2012 07:05 PM
filetype: zip
filesize: 189.39kb
downloads: 219

Attached is a zip file containing everything you need to run FIRST Team 341's laptop-based vision system. We are very proud of this software, as it propelled us to 4 Regional/District Championships and the Curie Division Finals. Every shot our robot attempted in Rebound Rumble was completely controlled by this code.

To get it up and running on your computer (which can be Windows, Linux, or Mac OSX), you first should install the WPI SmartDashboard using the installer which you can find here: http://firstforge.wpi.edu/sf/frs/do/listReleases/projects.smartdashboard/frs.installer

Note that for Linux or OSX builds (or if you just want to use more recent libraries than are included in the SmartDashboard installer on Windows), you will need to do the installation by hand. The libraries you will need are OpenCV (http://opencv.willowgarage.com/wiki/), JavaCV (http://code.google.com/p/javacv/), and FFMPEG (http://ffmpeg.org/). Make sure that the OpenCV and JavaCV versions you install are compatible! Meaning the same version number of OpenCV, and the same architecture (x86 or x86-64). You then need to make sure the paths to the OpenCV libraries are in your system path.

To use the widget with the Axis camera, you should configure your camera to allow anonymous viewing and make sure it is connected directly to your wireless bridge (not the cRIO). The IP must also be set by right clicking on the SmartDashboard widget. For best results, you will also want to ensure that your camera has a very fast exposure time (we used 1/120s). The FIRST control system documentation and vision whitepaper go over how to do this.

You can also run the widget in a "stand alone" fashion, as the class has a Java main method that you can use to load images from disk and display the processed results. Sample images are included.

Our vision system uses OpenCV library functions to successively filter the raw camera image and find the vision targets. The highest target detected is then used to compute range and bearing information. With a statically mounted camera, we use inverse trigonometry to find the distance to the target based on the height in the image. Bearing is computed by adding the offset of the target horizontally in the camera frame to the heading we are getting back from the robot. This lets us use a closed loop controller on the robot to use the gyro to "aim". If your robot has a turret, you could instead send the turret angle back to the SmartDashboard and get the same results. The computed range is used to find an optimal shooter RPM, which also gets sent back to the robot. We interpolate between points in our "range-to-RPM" lookup table in order to provide smooth outputs to our shooter wheel speed control loop.

Thanks to Brad Miller, WPI, Kevin O'Connor, and the FRC Control System team for making OpenCV-based vision processing a reality in 2012!


------------------------------------------------
hodgepodge 05-02-2012 09:40 PM

It's great to see how others approached the vision processing. I wrote the vision code for team 118 this year and from a quick glance, your algorithm is quite similar.

Our code ran onboard on a BeagleBone running Linux and straight OpenCV, written in C++. libCurl was used to capture images and a UDP socket spewed angle and distance data to the cRio.


------------------------------------------------
Jared341 05-09-2012 10:14 AM

    Originally Posted by Leeebowitz 
    Very cool! I just finished taking an online data structures course from IIT, so it was really awesome to see the implementation of the TreeMap! Just so that I'm sure I understand, it is used to quickly find an rpm based on your distance, right? What are the advantages (if any) of using this instead of plugging your data points into an excel graph and getting a polynomial best fit equation to transform your distance into an rpm?

The biggest advantage was it made tuning very easy. Take the bot to the practice field with some balls, fire away, and record the range/RPM information into the TreeMap if you like what you saw (we actually had a version of the software that let the operator do this with the push of a "commit" button, but we ended up doing it all by hand just to sanitize the values). No need to fire up Excel.


------------------------------------------------
Jared341 05-09-2012 10:16 AM
    Originally Posted by Tom Bottiglieri 
    I suspect if vision is a part of upcoming games, we will probably use a solution very similar to this. This year we wrote all of our vision code on top of WPILib/NIVision to run on the cRIO. In the end we got this to work pretty well, but development and debugging was a bit of a pain compared to your system.

By far the biggest advantage of a laptop-based solution was the development process that it facilitated. Simply collect some representative images and then you can go off and tune your vision software on a laptop without needing a cRIO or the FRC control system.


------------------------------------------------
Jared341 02-08-2013 10:50 AM
 
It looks like some of the internal changes to SmartDashboard for 2013 have broken stand-alone operation. Never fear, here is how to fix it:

Add the line:
Code:

    DashboardFrame frame = new DashboardFrame(false);

...inside the main method before creating the DaisyCVWidget.


------------------------------------------------
