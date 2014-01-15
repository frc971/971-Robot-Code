package aos;

import java.nio.ByteBuffer;

/**
 * Retrieves images from the queue system.<br>
 * {@link #getTimestamp()} returns the value from the v4l2 driver,
 * which is a CLOCK_MONOTONIC time for most (all?) of them and definitely uvcvideo.
 */
public class QueueImageGetter extends JPEGImageGetter {
	private final long nativeID;
	public QueueImageGetter() {
		nativeID = Natives.queueInit();
	}
	
	@Override
	public ByteBuffer getJPEG() {
		final ByteBuffer buf = Natives.queueGetJPEG(nativeID);
		if (buf == null) {
			return null;
		}
		return buf.asReadOnlyBuffer();
	}

	@Override
	public void release() {
		Natives.queueReleaseJPEG(nativeID);
	}

	@Override
	public double getTimestamp() {
		return Natives.queueGetTimestamp(nativeID);
	}
}
