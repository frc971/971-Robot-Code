/**
 * 
 */
package org.frc971;

/**
 * @author daniel
 *
 */

//get debug images for Java camera processor

import javax.imageio.ImageIO;

import edu.wpi.first.wpijavacv.WPIColorImage;

import java.io.File;
import java.io.IOException;

public class TestImageGetter {
	private String path_to_images;
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
	
	private String cocatenate_paths(String path1, String path2) {
		if (path1.charAt(path1.length() - 1) == '/')
			return path1 + path2;
		else
			return path1 + "/" + path2;
	}
	public TestImageGetter(String path_to_images) {
		this.path_to_images = path_to_images;
	}
	public WPIColorImage GetNext() {
		image_index++;
		if (image_index < images.length) {
			String image_to_get = images[image_index];
			try {
				current_image = new WPIColorImage(ImageIO.read(new File(cocatenate_paths(path_to_images, image_to_get))));
				return current_image;
			}
			catch (IOException e) {
				System.err.println("Could not open file.");
				return null;
			}
		}
		else
			image_index--;
			return null;
	}
	public WPIColorImage GetPrev() {
		image_index--;
		if (image_index >= 0) {
			String image_to_get = images[image_index];
			try {
				current_image = new WPIColorImage(ImageIO.read(new File(cocatenate_paths(path_to_images, image_to_get))));
				return current_image;
			}
			catch (IOException e) {
				System.err.println("Could not open file.");
				return null;
			}
		}
		else
			image_index++;
			return null;
	}
	public WPIColorImage GetCurrent() {
		return current_image;
	}
}
