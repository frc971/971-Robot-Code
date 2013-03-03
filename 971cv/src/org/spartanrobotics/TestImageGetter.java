/**
 * 
 */
package org.spartanrobotics;

/**
 * @author daniel
 *
 */

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;

import java.util.logging.Logger;

import javax.imageio.ImageIO;

import edu.wpi.first.wpijavacv.WPIColorImage;

/** Get debug images for Java camera processor. */
public class TestImageGetter implements ImageGetter{
	
	private final static Logger LOG = Logger.getLogger(
			TestImageGetter.class.getName());
	
	private WPIColorImage[] loadedImages;
	
	private int imageIndex = -1;
	private String currentName;
	
	/** Gets the name to display at the top of the image window. */
	public String getName() {
		return currentName;
	}
	
	/** Constructor
	 * 
	 * @param path_to_images is the path to the directory where our images are.
	 * @throws IOException 
	 */
	public TestImageGetter(String path_to_images) {
		File directory = new File(path_to_images);
		loadedImages = new WPIColorImage[directory.listFiles().length];
		
		//pre-load all the images
		int i = 0;
		for (final File fileEntry : directory.listFiles()) {
			try {
				BufferedImage image = ImageIO.read(fileEntry);
				if (image != null) {
					loadedImages[i] = new WPIColorImage(image);
				} else {
					//we attempted to load what was not an image. Skip it
					LOG.info("Preloading debug images; skipping incompatible: " + fileEntry.getName());
					continue;
				}
			} catch (IOException e) {
				//we couldn't open a file. Skip it
				LOG.info("Preloading debug images; skipping unopenable: " + fileEntry.getName());
				continue;
			}
			
			currentName = fileEntry.getName();
			++i;
		}
	}
	
	/** Gets the next debugging image.
	 * 
	 * @return Returns the next test image.
	 */
	public WPIColorImage getFrame() {
		++imageIndex;
		if (imageIndex < loadedImages.length) {
			return loadedImages[imageIndex];
		} else {
			imageIndex = loadedImages.length - 1;
			return loadedImages[imageIndex];
		}
	}
	
	/** Gets the previous debugging image.
	 * 
	 * @return Returns the previous test image.
	 */
	public WPIColorImage getPrev() {
		--imageIndex;
		if (imageIndex > 0) {
			return loadedImages[imageIndex];
		} else {
			imageIndex = 0;
			return loadedImages[imageIndex];
		}
	}
}
