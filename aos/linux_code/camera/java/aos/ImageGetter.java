package aos;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

/**
 * An object that can retrieve images from somewhere.
 */
public interface ImageGetter {
	public static int width = 640, height = 480;
	
	/**
	 * Gets an image.
	 * @param out Where to write the image to. Must be a 3-channel {@link opencv_core#IPL_DEPTH_8U} image.
	 * @return whether it succeeded or not
	 */
	public boolean get(IplImage out);
	/**
	 * Only valid after a successful {@link #get()}.
	 * @return The timestamp from the most recent frame. Will be in seconds with at least ms accuracy.
	 */
	public double getTimestamp();
}

