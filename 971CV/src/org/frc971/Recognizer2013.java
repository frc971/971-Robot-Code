package org.frc971;

import java.util.ArrayList;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvSize;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;
import com.googlecode.javacv.cpp.opencv_imgproc.IplConvKernel;

import edu.wpi.first.wpijavacv.DaisyExtensions;
import edu.wpi.first.wpijavacv.WPIBinaryImage;
import edu.wpi.first.wpijavacv.WPIColor;
import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIContour;
import edu.wpi.first.wpijavacv.WPIImage;
import edu.wpi.first.wpijavacv.WPIPoint;
import edu.wpi.first.wpijavacv.WPIPolygon;

/**
 * Vision target recognizer for FRC 2013.
 *
 * @author jerry
 */
public class Recognizer2013 implements Recognizer {

    // --- Constants that need to be tuned.
    static final double kRoughlyHorizontalSlope = Math.tan(Math.toRadians(30));
    static final double kRoughlyVerticalSlope = Math.tan(Math.toRadians(90 - 30));
    private int min1Hue;
    private int max1Hue;
    private int min1Sat;
    private int min1Val;
    static final int kHoleClosingIterations = 2;
    static final double kPolygonPercentFit = 12;

    static final int kMinWidthAt320 = 35; // for high goal and middle goals

    // These aspect ratios include the outside edges of the vision target tape.
    static final double kHighGoalAspect = (21 + 8.0) / (54 + 8);
    static final double kMiddleGoalAspect = (24 + 8.0) / (54 + 8);
    static final double kMinAspect = kHighGoalAspect * 0.6;
    static final double kMaxAspect = kMiddleGoalAspect * 1.4;

    static final double kShooterOffsetDeg = 0;
    static final double kHorizontalFOVDeg = 47.0;
    static final double kVerticalFOVDeg = 480.0 / 640.0 * kHorizontalFOVDeg;

    // --- Colors for drawing indicators on the image.
    private static final WPIColor reject1Color = WPIColor.GRAY;
    private static final WPIColor reject2Color = WPIColor.YELLOW;
    private static final WPIColor candidateColor = WPIColor.BLUE;
    private static final WPIColor targetColor = new WPIColor(255, 0, 0);

    // Show intermediate images for parameter tuning.
    private final DebugCanvas thresholdedCanvas = new DebugCanvas("thresholded");
    private final DebugCanvas morphedCanvas = new DebugCanvas("morphed");

    // Data to reuse for each frame.
    private final DaisyExtensions daisyExtensions = new DaisyExtensions();
    private final IplConvKernel morphKernel = IplConvKernel.create(3, 3, 1, 1,
            opencv_imgproc.CV_SHAPE_RECT, null);
    private final ArrayList<WPIPolygon> polygons = new ArrayList<WPIPolygon>();

    // Frame-size-dependent data to reuse for each frame.
    private CvSize size = null;
    private WPIColorImage rawImage;
    private IplImage bin;
    private IplImage hsv;
    private IplImage hue;
    private IplImage sat;
    private IplImage val;
    private int minWidth;
    private WPIPoint linePt1, linePt2; // crosshair endpoints

    public Recognizer2013() {
        setHSVRange(70, 106, 137, 27);
    }

    @Override
    public void setHSVRange(int minHue, int maxHue, int minSat, int minVal) {
        min1Hue = minHue - 1; // - 1 because cvThreshold() does > instead of >=
        max1Hue = maxHue + 1;
        min1Sat = minSat - 1;
        min1Val = minVal - 1;
    }
    @Override
    public int getHueMin() { return min1Hue + 1; }
    @Override
    public int getHueMax() { return max1Hue - 1; }
    @Override
    public int getSatMin() { return min1Sat - 1; }
    @Override
    public int getValMin() { return min1Val - 1; }

    @Override
    public void showIntermediateStages(boolean enable) {
        thresholdedCanvas.show = enable;
        morphedCanvas.show = enable;
    }

    @Override
    public WPIImage processImage(WPIColorImage cameraImage) {
        // (Re)allocate the intermediate images if the input is a different
        // size than the previous image.
        if (size == null || size.width() != cameraImage.getWidth()
                || size.height() != cameraImage.getHeight()) {
            size = opencv_core.cvSize(cameraImage.getWidth(),
                    cameraImage.getHeight());
            rawImage = DaisyExtensions.makeWPIColorImage(
                    DaisyExtensions.getIplImage(cameraImage));
            bin = IplImage.create(size, 8, 1);
            hsv = IplImage.create(size, 8, 3);
            hue = IplImage.create(size, 8, 1);
            sat = IplImage.create(size, 8, 1);
            val = IplImage.create(size, 8, 1);
            minWidth = (kMinWidthAt320 * cameraImage.getWidth() + 319) / 320;

            int horizontalOffsetPixels = (int)Math.round(
                    kShooterOffsetDeg * size.width() / kHorizontalFOVDeg);
            int x = size.width() / 2 + horizontalOffsetPixels;
            linePt1 = new WPIPoint(x, size.height() - 1);
            linePt2 = new WPIPoint(x, 0);
        } else {
            // Copy the camera image so it's safe to draw on.
            opencv_core.cvCopy(DaisyExtensions.getIplImage(cameraImage),
                    DaisyExtensions.getIplImage(rawImage));
        }

        IplImage input = DaisyExtensions.getIplImage(rawImage);

        // Threshold the pixels in HSV color space.
        // TODO(jerry): Do this in one pass of a pixel-processing loop.
        opencv_imgproc.cvCvtColor(input, hsv, opencv_imgproc.CV_BGR2HSV_FULL);
        opencv_core.cvSplit(hsv, hue, sat, val, null);

        // NOTE: Since red is at the end of the cyclic color space, you can OR
        // a threshold and an inverted threshold to match red pixels.
        // TODO(jerry): Use tunable constants instead of literals.
        opencv_imgproc.cvThreshold(hue, bin, min1Hue, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(hue, hue, max1Hue, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(sat, sat, min1Sat, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(val, val, min1Val, 255, opencv_imgproc.CV_THRESH_BINARY);

        // Combine the results to obtain a binary image which is mostly the
        // interesting pixels.
        opencv_core.cvAnd(hue, bin, bin, null);
        opencv_core.cvAnd(bin, sat, bin, null);
        opencv_core.cvAnd(bin, val, bin, null);

        thresholdedCanvas.showImage(bin);

        // Fill in gaps using binary morphology.
        opencv_imgproc.cvMorphologyEx(bin, bin, null, morphKernel,
                opencv_imgproc.CV_MOP_CLOSE, kHoleClosingIterations);

        morphedCanvas.showImage(bin);

        // Find contours.
        //
        // TODO(jerry): Request contours as a two-level hierarchy (blobs and
        // holes)? The targets have known sizes and their holes have known,
        // smaller sizes. This matters for distance measurement. OTOH it's moot
        // if/when we use the vertical stripes for distance measurement.
        WPIBinaryImage binWpi = DaisyExtensions.makeWPIBinaryImage(bin);
        WPIContour[] contours = daisyExtensions.findConvexContours(binWpi);

        // Simplify the contours to polygons and filter by size and aspect ratio.
        //
        // TODO(jerry): Also look for the two vertical stripe vision targets.
        // They'll greatly increase the precision of measuring the distance. If
        // both stripes are visible, they'll increase the accuracy for
        // identifying the high goal.
        polygons.clear();
        for (WPIContour c : contours) {
            if (c.getWidth() >= minWidth) {
                double ratio = ((double) c.getHeight()) / c.getWidth();
                if (ratio >= kMinAspect && ratio <= kMaxAspect) {
                    polygons.add(c.approxPolygon(kPolygonPercentFit));
                    //        	    System.out.println("  Accepted aspect ratio " + ratio);
                } else {
                    //        	    System.out.println("  Rejected aspect ratio " + ratio);
                }
            }
        }

        // Pick the target with the highest center-point that matches yet more
        // filter criteria.
        WPIPolygon bestTarget = null;
        int highestY = Integer.MAX_VALUE;

        for (WPIPolygon p : polygons) {
            // TODO(jerry): Replace boolean filters with a scoring function?
            if (p.isConvex() && p.getNumVertices() == 4) { // quadrilateral
                WPIPoint[] points = p.getPoints();
                // Filter for polygons with 2 ~horizontal and 2 ~vertical sides.
                int numRoughlyHorizontal = 0;
                int numRoughlyVertical = 0;
                for (int i = 0; i < 4; ++i) {
                    double dy = points[i].getY() - points[(i + 1) % 4].getY();
                    double dx = points[i].getX() - points[(i + 1) % 4].getX();
                    double slope = Double.MAX_VALUE;
                    if (dx != 0) {
                        slope = Math.abs(dy / dx);
                    }

                    if (slope < kRoughlyHorizontalSlope) {
                        ++numRoughlyHorizontal;
                    } else if (slope > kRoughlyVerticalSlope) {
                        ++numRoughlyVertical;
                    }
                }

                if (numRoughlyHorizontal >= 2 && numRoughlyVertical == 2) {
                    rawImage.drawPolygon(p, candidateColor, 2);

                    int pCenterX = p.getX() + p.getWidth() / 2;
                    int pCenterY = p.getY() + p.getHeight() / 2;

                    rawImage.drawPoint(new WPIPoint(pCenterX, pCenterY),
                            targetColor, 2);
                    if (pCenterY < highestY) {
                        bestTarget = p;
                        highestY = pCenterY;
                    }
                } else {
                    rawImage.drawPolygon(p, reject2Color, 1);
                }
            } else {
                rawImage.drawPolygon(p, reject1Color, 1);
            }
        }

        if (bestTarget != null) {
            double w = bestTarget.getWidth();
            double h = bestTarget.getHeight();
            double x = bestTarget.getX() + w / 2;
            double y = bestTarget.getY() + h / 2;

            rawImage.drawPolygon(bestTarget, targetColor, 2);

            System.out.println("Best target at (" + x + ", " + y + ") size "
                    + w + " x " + h);
        } else {
            System.out.println("No target found");
        }

        // Draw a crosshair
        rawImage.drawLine(linePt1, linePt2, targetColor, 1);

        daisyExtensions.releaseMemory();
        //System.gc();

        return rawImage;
    }

}
