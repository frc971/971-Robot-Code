package org.frc971;

import com.googlecode.javacv.CanvasFrame;
import com.googlecode.javacv.cpp.opencv_core.IplImage;

public class DebugCanvas {
    public boolean show;
    private CanvasFrame canvasFrame;
    private String name;

    public DebugCanvas(String name) {
	this.name = name;
    }

    public void showImage(IplImage image) {
        if (show) {
            if (canvasFrame == null) {
        	canvasFrame = new CanvasFrame(name);
            }
            canvasFrame.setName(name);
            canvasFrame.showImage(image.getBufferedImage());
        } else {
            if (canvasFrame != null) {
        	canvasFrame.dispose();
        	canvasFrame = null;
            }
        }
    }
}
