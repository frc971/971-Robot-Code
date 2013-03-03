/**
 * 
 */
package org.spartanrobotics;

import edu.wpi.first.wpijavacv.WPIColorImage;

/**
 * @author daniel
 *
 */

/** Interface for program image contributors. */
public interface ImageGetter {
	
	/** Gets the next image from the source/ */
	WPIColorImage getFrame();
	
	/** Gets the name of the image source. */
	String getName();
	
}
