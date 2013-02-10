package aos;

import java.nio.ByteBuffer;

import com.googlecode.javacv.cpp.opencv_core.IplImage;

/**
 * Helper class for {@link ImageGetter}s that return JPEG images.
 */
public abstract class JPEGImageGetter implements ImageGetter {
	
	private final JPEGDecoder decoder = new JPEGDecoder();

	@Override
	public boolean get(IplImage out) {
		final ByteBuffer jpeg = getJPEG();
		if (jpeg == null) return false;
		final boolean r = decoder.decode(jpeg, out);
		release();
		return r;
	}
	
	protected abstract ByteBuffer getJPEG();
	protected void release() {}

}
