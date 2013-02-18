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

    // Constants that need to be tuned
    static final double kRoughlyHorizontalSlope = Math.tan(Math.toRadians(20));
    static final double kRoughlyVerticalSlope = Math.tan(Math.toRadians(90 - 20));
    static final int kMinWidth = 20;
    static final int kMaxWidth = 400;
    static final int kHoleClosingIterations = 9;

    static final double kShooterOffsetDeg = 0;
    static final double kHorizontalFOVDeg = 47.0;
    static final double kVerticalFOVDeg = 480.0 / 640.0 * kHorizontalFOVDeg;

    // Colors for drawing indicators on the image.
    private static final WPIColor reject1Color = WPIColor.GRAY;
    private static final WPIColor reject2Color = WPIColor.YELLOW;
    private static final WPIColor candidateColor = WPIColor.BLUE;
    private static final WPIColor targetColor = new WPIColor(255, 0, 0);

    // Show intermediate images for parameter tuning.
    private final DebugCanvas thresholdedCanvas = new DebugCanvas("thresholded");
    private final DebugCanvas morphedCanvas = new DebugCanvas("morphed");

    // JavaCV data to reuse for each frame.
    private final DaisyExtensions daisyExtensions = new DaisyExtensions();
    private final IplConvKernel morphKernel = IplConvKernel.create(3, 3, 1, 1,
		opencv_imgproc.CV_SHAPE_RECT, null);
    private CvSize size = null;
    private WPIContour[] contours;
    private final ArrayList<WPIPolygon> polygons = new ArrayList<WPIPolygon>();
    private WPIColorImage rawImage;
    private IplImage bin;
    private IplImage hsv;
    private IplImage hue;
    private IplImage sat;
    private IplImage val;
    private WPIPoint linePt1, linePt2; // crosshair endpoints

    public Recognizer2013() {
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

            int horizontalOffsetPixels = (int)Math.round(
        	    kShooterOffsetDeg * size.width() / kHorizontalFOVDeg);
            int x = size.width() / 2 + horizontalOffsetPixels;
            linePt1 = new WPIPoint(x, size.height() - 1);
            linePt2 = new WPIPoint(x, 0);
        } else {
            opencv_core.cvCopy(DaisyExtensions.getIplImage(cameraImage),
        	    DaisyExtensions.getIplImage(rawImage));
        }

        IplImage input = DaisyExtensions.getIplImage(rawImage);

        // Threshold the pixels in HSV color space.
        // TODO(jerry): Do this in one pass of a pixel-processing loop.
        opencv_imgproc.cvCvtColor(input, hsv, opencv_imgproc.CV_BGR2HSV);
        opencv_core.cvSplit(hsv, hue, sat, val, null);

        // NOTE: Since red is at the end of the cyclic color space, you can OR
        // a threshold and an inverted threshold to match red pixels.
        // TODO(jerry): Use tunable constants instead of literals.
        opencv_imgproc.cvThreshold(hue, bin, 60 - 15, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(hue, hue, 60 + 15, 255, opencv_imgproc.CV_THRESH_BINARY_INV);
        opencv_imgproc.cvThreshold(sat, sat, 200, 255, opencv_imgproc.CV_THRESH_BINARY);
        opencv_imgproc.cvThreshold(val, val, 55, 255, opencv_imgproc.CV_THRESH_BINARY);

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
        WPIBinaryImage binWpi = DaisyExtensions.makeWPIBinaryImage(bin);
        contours = daisyExtensions.findConvexContours(binWpi);

        // Simplify the contour to polygons and filter by size and aspect ratio.
        // TODO(jerry): Use tunable constants instead of literals.
        polygons.clear();
        for (WPIContour c : contours) {
            double ratio = ((double) c.getHeight()) / ((double) c.getWidth());
            if (ratio < 1.0 && ratio > 0.5 && c.getWidth() >= kMinWidth
        	    && c.getWidth() <= kMaxWidth) {
                polygons.add(c.approxPolygon(20));
            }
        }

        // Pick the highest target that matches more filter criteria.
        WPIPolygon bestTarget = null;
        int highestY = Integer.MAX_VALUE;

        for (WPIPolygon p : polygons) {
            if (p.isConvex() && p.getNumVertices() == 4) { // quadrilateral
                WPIPoint[] points = p.getPoints();
                // We expect the polygon to have a top line that is nearly
                // horizontal and two side lines that are nearly vertical.
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

                if (numRoughlyHorizontal >= 1 && numRoughlyVertical == 2) {
                    rawImage.drawPolygon(p, candidateColor, 2);

                    int pCenterX = p.getX() + p.getWidth() / 2;
                    int pCenterY = p.getY() + p.getHeight() / 2;

                    rawImage.drawPoint(new WPIPoint(pCenterX, pCenterY),
                	    candidateColor, 3);
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
