package org.frc971;

import edu.wpi.first.wpijavacv.WPIColorImage;

/**
 * Vision target recognizer.
 *
 * @author jerry
 */
public interface Recognizer {

    /**
     * Sets the HSV filter to allow H in [minHue .. maxHue], S >= minSat,
     * V >= minVal.
     */
    void setHSVRange(int minHue, int maxHue, int minSat, int minVal);

    int getHueMin();
    int getHueMax();
    int getSatMin();
    int getValMin();

    /** Enables/disables windows to view intermediate stages, for tuning. */
    void showIntermediateStages(boolean enable);

    /**
     * Processes a camera image, returning an image to display for targeting
     * and debugging, e.g. with cross-hairs and marked targets.
     *<p>
     * SIDE EFFECTS: May modify cameraImage.
     */
    Target processImage(WPIColorImage cameraImage);
}
