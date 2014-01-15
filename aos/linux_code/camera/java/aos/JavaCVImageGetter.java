package aos;

import java.util.logging.Level;
import java.util.logging.Logger;

import com.googlecode.javacv.FrameGrabber;
import com.googlecode.javacv.cpp.opencv_core.IplImage;


/**
 * Adapts between the JavaCV {@link FrameGrabber} and {@link ImageGetter}.
 * There is (at least) 1 extra copy involved, so this shouldn't be used if you care about speed.
 */
public class JavaCVImageGetter implements ImageGetter {
	private static final Logger LOG = Logger.getLogger(JavaCVImageGetter.class
			.getName());
	private final FrameGrabber grabber;
	
	public JavaCVImageGetter(FrameGrabber grabber) {
		this.grabber = grabber;
		if (grabber.getImageWidth() != width || grabber.getImageHeight() != height) {
			if (grabber.getImageWidth() == 0 && grabber.getImageHeight() == 0) {
				LOG.log(Level.WARNING, "grabber says it will give 0x0 images at the start. ignoring");
			} else {
				throw new IllegalArgumentException("grabber says it will give images that are the wrong size!!");
			}
		}
	}

	@Override
	public boolean get(IplImage out) {
		try {
			final IplImage frame = grabber.grab();
			if (grabber.getImageWidth() != width || grabber.getImageHeight() != height) {
				LOG.log(Level.SEVERE, "grabber says it will give the wrong size images");
				return false;
			}
			if (out.imageSize() != frame.imageSize()) {
				LOG.log(Level.SEVERE, "the grabber gave a " + frame.imageSize() + "-byte image" +
						"but a " + out.imageSize() + "-byte image was passed in");
				return false;
			}
			out.getByteBuffer().put(frame.getByteBuffer());
			return true;
		} catch (FrameGrabber.Exception e) {
			LOG.log(Level.WARNING, "grabber.grab() threw an exception", e);
			return false;
		}
	}

	@Override
	public double getTimestamp() {
		// grabber.getTimestamp seems to be in ms (all the implementations are at least)
		return grabber.getTimestamp() / 1000.0;
	}
}
