package aos;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

public class Thresholder {
	/**
	 * Thresholds in into out.
	 * All of the int arguments should be unsigned bytes except hoffset, which should be a signed byte.
	 * The min and max parameters specify what colors to accept.
	 * @param in The image to threshold. Must be a 3-channel {@link opencv_core#IPL_DEPTH_8U} image in the HSV color space.
	 * @param out Where to write the thresholded image to. Must be a 1-channel {@link opencv_core#IPL_DEPTH_8U} image.
	 * @param hoffset An offset to be added to the hue value before comparing it to {@code hmin} and {@code hmax}.
	 * The addition will be performed to a {@code uint8_t}, which will wrap around. This means that it must be positive.
	 * Useful for finding red values.
	 */
	public static void threshold(IplImage in, IplImage out, int hoffset, int hmin, int hmax,
			int smin, int smax, int vmin, int vmax) {
		if (in.nChannels() != 3 || in.depth() != opencv_core.IPL_DEPTH_8U) {
			throw new IllegalArgumentException("in is invalid");
		}
		if (out.nChannels() != 1 || out.depth() != opencv_core.IPL_DEPTH_8U) {
			throw new IllegalArgumentException("out is invalid");
		}
		Natives.threshold(in.getByteBuffer(), out.getByteBuffer(), (short)hoffset,
				(char)hmin, (char)hmax, (char)smin, (char)smax, (char)vmin, (char)vmax);
	}
}
