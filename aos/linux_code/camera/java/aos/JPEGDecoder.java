package aos;

import java.nio.ByteBuffer;

import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

/**
 * Efficiently decodes a JPEG image from a @{class ByteBuffer} into an @{class IplImage}.
 * Instances are not safe for use from multiple threads.
 * The first use of an instance allocates some largish buffers which are never freed. 
 */
public class JPEGDecoder {
	private final long[] state = new long[1];
	
	/**
	 * @param in Must be direct. The {@link ByteBuffer#limit()} of it will be respected.
	 * @param out Where to write the decoded image to. Must be a 3-channel {@link opencv_core#IPL_DEPTH_8U} image.
	 * Will be written in the RGB color space.
	 * @return Whether or not it succeeded. If not, {@code out} is undefined.
	 */
	public boolean decode(ByteBuffer in, IplImage out) {
		if (out.nChannels() != 3 || out.depth() != opencv_core.IPL_DEPTH_8U) {
			throw new IllegalArgumentException("out is invalid");
		}
		return Natives.decodeJPEG(state, in, in.limit(), out.getByteBuffer());
	}
}

