package org.frc971;

import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;

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
    public void setHSVRange(int minHue, int maxHue, int minSat, int minVal);

    public int getHueMin();
    public int getHueMax();
    public int getSatMin();
    public int getValMin();

    /**
     * Processes a camera image, returning an image to display for targeting
     * and debugging, e.g. with cross-hairs and marked targets.
     *<p>
     * SIDE EFFECTS: May modify cameraImage.
     */
    WPIImage processImage(WPIColorImage cameraImage);
}
