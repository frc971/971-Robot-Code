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
     * Processes a camera image, returning an image to display for targeting
     * and debugging, e.g. with cross-hairs and marked targets.
     *<p>
     * SIDE EFFECTS: May modify cameraImage.
     */
    WPIImage processImage(WPIColorImage cameraImage);
}
