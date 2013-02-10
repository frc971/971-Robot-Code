package aos;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Queue;
import java.util.logging.Logger;

import com.googlecode.javacv.cpp.opencv_core.IplImage;

/**
 * Provides {@link IplImage}s that can be used with a {@link DebugServer}.
 */
public class ServableImage {
	@SuppressWarnings("unused")
	private static final Logger LOG = Logger.getLogger(ServableImage.class
			.getName());
	private final int width, height, depth, channels;
	private final ArrayList<Queue<IplImage>> queues = new ArrayList<Queue<IplImage>>();
	private final ArrayList<IplImage> snapshots = new ArrayList<IplImage>();
	private final IplImage current;
	private int debugging = 0;

	public ServableImage(int width, int height, int depth, int channels) {
		this.width = width;
		this.height = height;
		this.depth = depth;
		this.channels = channels;

		current = IplImage.create(width, height, depth, channels);
	}

	/**
	 * @return the number of bytes in each image
	 */
	public int imageSize() {
		return width * height * depth * channels / 8;
	}

	/**
	 * @return the number of bits in each pixel
	 */
	public short bitsPerPixel() {
		return (short) (depth * channels);
	}

	/**
	 * Retrieves an image that should be used for debugging. It clears the value
	 * when called. {@link #releastSnapshot} MUST be called with the result.
	 * 
	 * @param i
	 *            Which snapshot to retrieve. 0 means the most recent final
	 *            image.
	 * @return The most recent image at this index. {@code null} if there isn't
	 *         a new one since last time this function was called. Will be in
	 *         the correct color space to dump into a BMP image if this is a
	 *         3-channel image.
	 */
	public synchronized IplImage getSnapshot(int i) {
		if (snapshots.size() > i) {
			return snapshots.get(i);
		} else {
			return null;
		}
	}

	public synchronized void releaseSnapshot(int i, IplImage image) {
		queues.get(i).add(image);
	}

	/**
	 * This function will return the same image if called repeatedly until
	 * {@link #releaseImage} is called.
	 * 
	 * @return the current image
	 */
	public synchronized IplImage getImage() {
		return current;
	}

	/**
	 * Releases the current image (to be potentially sent out for debugging and
	 * then reused).
	 */
	public synchronized void releaseImage() {
		recordSnapshot(0);
	}

	/**
	 * Records a copy of the current image for debugging. It will be accessible
	 * at <{@code <base path>?i=<the value>}>. Does nothing unless
	 * {@link #isDebugging()}. This method <i>should</i> get called regardless
	 * of {@link #isDebugging()} to avoid outputting old debugging images. Note:
	 * 0 is not a valid snapshot number.
	 * 
	 * @param i
	 *            which snapshot this is
	 */
	public synchronized void recordSnapshot(int i) {
		while (queues.size() <= i) {
			queues.add(null);
		}
		if (queues.get(i) == null) {
			queues.set(i, new ArrayDeque<IplImage>());
		}
		while (snapshots.size() <= i) {
			snapshots.add(null);
		}
		if (snapshots.get(i) != null) {
			releaseSnapshot(i, snapshots.get(i));
		}
		if (isDebugging()) {
			IplImage snapshot = queues.get(i).poll();
			if (snapshot == null) {
				snapshot = IplImage.create(width, height, depth, channels);
			}
			if (channels == 3) {
				Natives.convertBGR2BMP(current.getByteBuffer(),
						snapshot.getByteBuffer());
			} else {
				snapshot.getByteBuffer().put(current.getByteBuffer());
			}
			snapshots.set(i, snapshot);
		} else {
			snapshots.set(i, null);
		}
	}

	/**
	 * @return whether or not to do extra debug work with the current image
	 */
	public synchronized boolean isDebugging() {
		return debugging > 0;
	}

	/**
	 * Whoever turns this on should turn it off when they're done.
	 */
	synchronized void setDebugging(boolean debugging) {
		if (debugging) {
			++this.debugging;
		} else {
			--this.debugging;
		}
	}
}
