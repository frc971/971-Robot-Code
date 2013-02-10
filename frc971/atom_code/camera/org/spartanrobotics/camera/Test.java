package org.spartanrobotics.camera;

import java.io.IOException;
import com.googlecode.javacv.cpp.opencv_core;
import com.googlecode.javacv.cpp.opencv_core.CvPoint;
import com.googlecode.javacv.cpp.opencv_core.CvScalar;
import com.googlecode.javacv.cpp.opencv_core.IplImage;
import com.googlecode.javacv.cpp.opencv_imgproc;

import aos.CameraProcessor;
import aos.DebugServer;
import aos.ImageGetter;
import aos.ServableImage;
import aos.Thresholder;

public class Test extends CameraProcessor {
	private final DebugServer server = new DebugServer(9719);
	private final IplImage hsv = IplImage.create(ImageGetter.width, ImageGetter.height, opencv_core.IPL_DEPTH_8U, 3);
	private final ServableImage thresholded = new ServableImage(ImageGetter.width, ImageGetter.height, opencv_core.IPL_DEPTH_8U, 1);
	
	private Test(String[] args) throws IOException {
		super(args);
		
		server.addImage("/start", start);
		server.addImage("/thresholded", thresholded, new DebugServer.Palette().add(0, 0, 0, 0).add(0, 0xFF, 0, 0xFF).add(0, 0, 0xFF, 0xFF));
	}

	@Override
	protected void RunIteration() {
		server.setTimestamp(getter.getTimestamp());
		
		opencv_imgproc.cvCvtColor(start.getImage(), hsv, opencv_imgproc.CV_RGB2HSV);
		Thresholder.threshold(hsv, thresholded.getImage(), 0, 100, 160, 0, 255, 0, 255);
		thresholded.recordSnapshot(1);
		if (thresholded.isDebugging()) {
			opencv_core.cvCircle(thresholded.getImage(), new CvPoint(200, 200), 50, new CvScalar(2, 2, 2, 2), 5, 8, 0);
		}
		thresholded.releaseImage();
	}
	
	public static void main(String[] args) throws IOException {
		new Test(args).Run();
	}
}
