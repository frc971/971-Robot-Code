/**
 * 
 */
package org.frc971;

/**
 * @author daniel
 *
 */

import java.io.File;
import java.io.IOException;

import java.util.logging.Logger;

import javax.imageio.ImageIO;

import edu.wpi.first.wpijavacv.WPIColorImage;

/** Get debug images for Java camera processor. */
public class TestImageGetter {
	
	private String path_to_images;
	
	private final static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
	
	/** The names of our debugging images, without paths.
	 * The GetNext method should be used to get the first
	 * image, and not GetCurrent. */
	final static String[] images = {"45in_DoubleGreen.jpg",
									"57inLargeTarget_DoubleGreenBK.jpg",
									"FullField_DoubleGreenBK3.jpg",
									"FullField_SmallGreen.jpg",
									"HybridLine_DoubleGreenBK2.jpg",
									"HybridLine_DoubleGreenBK3.jpg",
									"HybridLine_DoubleGreenBK4.jpg",
									"HybridLine_SmallGreen2.jpg",
									"HybridLine_SmallGreen3.jpg",
									"HybridLine_SmallGreen4.jpg",
									"Midfield_DoubleGreenBK2.jpg",
									"Midfield_SmallGreen2.jpg",
									"Midfield_SmallGreen3.jpg",
									"Midfield_SmallGreen4.jpg",
									"OppLine_DoubleGreenBK2.jpg",
									"OppLine_SmallGreen2.jpg",
									"PyramidRight_DoubleGreenBK2.jpg",
									"PyramidRight_SmallGreen2.jpg"
									};
	
	private int image_index = -1;
	
	private WPIColorImage current_image = null;
	
	/** Helper method to concatenate paths, similar to Python's os.path.join(). */
	private String cocatenate_paths(String path1, String path2) {
		if (path1.charAt(path1.length() - 1) == '/')
			return path1 + path2;
		else
			return path1 + "/" + path2;
	}
	
	/** Constructor
	 * 
	 * @param path_to_images is the path to the directory where our images are.
	 */
	public TestImageGetter(String path_to_images) {
		this.path_to_images = path_to_images;
	}
	
	/** Gets the next debugging image.
	 * 
	 * @return Returns a WPIColorImage.
	 */
	public WPIColorImage GetNext() {
		image_index++;
		if (image_index < images.length) {
			String image_to_get = images[image_index];
			try {
				current_image = new WPIColorImage(ImageIO.read(new File(cocatenate_paths(path_to_images, image_to_get))));
				return current_image;
			}
			catch (IOException e) {
				LOG.warning("Could not open file.");
				return null;
			}
		}
		else
			image_index--;
			return null;
	}
	
	/** Gets the previous debugging image.
	 * 
	 * @return Returns a WPIColorImage.
	 */
	public WPIColorImage GetPrev() {
		image_index--;
		if (image_index >= 0) {
			String image_to_get = images[image_index];
			try {
				current_image = new WPIColorImage(ImageIO.read(new File(cocatenate_paths(path_to_images, image_to_get))));
				return current_image;
			}
			catch (IOException e) {
				LOG.warning("Could not open file.");
				return null;
			}
		}
		else
			image_index++;
			return null;
	}
	
	/** Gets the current debugging image. This is vestigial, it is not longer used.
	 * 
	 * @return Returns a WPIColorImage
	 */
	public WPIColorImage GetCurrent() {
		return current_image;
	}
}
