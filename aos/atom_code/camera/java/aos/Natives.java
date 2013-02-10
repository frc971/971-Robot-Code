package aos;

import com.googlecode.javacv.cpp.opencv_core;

import java.nio.ByteBuffer;
import java.util.logging.Level;

/**
 * <p>Package-private class that has all of the native functions in it to make them easier to implement.</p>
 * <p>WARNING: The raw native functions are <b>NOT</b> thread-safe!!!!! Any java functions that need to be thread safe MUST be synchronized in JAVA!</p>
 */
class Natives {
	static {
		NativeLoader.load("aos_camera");
		nativeInit(ImageGetter.width, ImageGetter.height);
	}
	private static native void nativeInit(int width, int height);
	/**
	 * Empty function to make sure the class gets loaded (which means loading the native library).
	 */
	public static void ensureLoaded() {}
	
	/**
	 * Decodes a JPEG from in into out. Both buffers must be direct.
	 * @param state a long[1] for storing thread-local state
	 * @param in the JPEG to decode
	 * @param inBytes how many bytes long the JPEG is
	 * @param out the buffer to write the decoded image into
	 * @return Whether or not it succeeded. If not, {@code out} is undefined.
	 */
	public static native boolean decodeJPEG(long[] state, ByteBuffer in, int inBytes, ByteBuffer out);

	/**
	 * Thresholds in into out. Both buffers must be direct.
	 * All of the short arguments should be unsigned bytes. The min and max parameters specify what colors to accept.
	 * @param in The image to threshold. Must be a 3-channel {@link opencv_core#IPL_DEPTH_8U} image buffer in the HSV color space.
	 * @param out Where to write the thresholded image to. Must be a 1-channel {@link opencv_core#IPL_DEPTH_8U} image buffer.
	 * @param hoffset An offset to be added to the hue value before comparing it to {@code hmin} and {@code hmax}.
	 * The addition will be performed to a {@code uint8_t}, which will wrap around. This means that it must be positive.
	 * Useful for finding red values.
	 */
	public static native void threshold(ByteBuffer in, ByteBuffer out, short hoffset, char hmin, char hmax,
			char smin, char smax, char vmin, char vmax);
	
	/**
	 * Converts the colors from in to the format required for dumping them into a BMP image.
	 * @param in The image to convert. Must be a 3-channel {@link opencv_core#IPL_DEPTH_8U} image buffer in the regular (BGR I think...) color space.
	 * @param out Where to write the converted image to. Must be a 3-channel {@link opencv_core#IPL_DEPTH_8U} image buffer.
	 */
	public static native void convertBGR2BMP(ByteBuffer in, ByteBuffer out);

	/**
	 * Retrieves a JPEG image from the queue system. Will block until a new one is ready.
	 * @param id from {@link #queueInit()}
	 * @return Will be direct. This buffer <b>must not EVER</b> be written to.
	 */
	public static native ByteBuffer queueGetJPEG(long id);
	/**
	 * Retrieves the latest frame timestamp from the queue system. Must only be called between {@link #queueGetJPEG} and {@link #queueReleaseJPEG}.
	 * @param id from {@link #queueInit()}
	 * @return a timestamp
	 */
	public static native double queueGetTimestamp(long id);
	/**
	 * Releases the last image retrieved from the queue system. The result of the last {@link #queueGetJPEG()} will now be invalid.
	 * @param id from {@link #queueInit()}
	 */
	public static native void queueReleaseJPEG(long id);
	/**
	 * Prepares to start retrieving JPEGs from the queues.
	 * @return the ID to pass to the other queue functions
	 */
	public static native long queueInit();
	
	/**
	 * Puts the given message into the logging framework.
	 * @param message the complete log message
	 * @param level the level (from {@link Level#intValue()}
	 */
	public static native void LOG(String message, int level);
}

