package org.frc971;

import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.WindowConstants;

import com.googlecode.javacv.CanvasFrame;

import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;

/* REQUIRED JAVA LIBRARIES:
 *   Program Files/SmartDashboard/
 *     extensions/lib/javacpp.jar
 *     extensions/lib/javacv-YOUR_OS.jar
 *     extensions/lib/javacv.jar
 *     extensions/lib/WPIJavaCV.jar
 *     SmartDashboard.jar                 -- maybe in the future
 *     extensions/WPICameraExtension.jar  -- maybe in the future
 *
 * REQUIRED NATIVE CODE LIBRARIES:
 *   Program Files/WPIJavaCV/
 *     JavaCV_2.2.0/javacv-bin/javacv-YOUR_OS.jar
 *     OpenCV_2.2.0/bin/*
 */
/**
 * FRC 2013 vision-target recognizer tuner app.
 *
 * @author jerry
 */
public class VisionTuner {
    private String[] testImageFilenames;
    private WPIColorImage[] testImages;
    private final CanvasFrame cameraFrame = new CanvasFrame("Camera");
    private int currentIndex = 0;
    private Recognizer recognizer = new Recognizer2013();

    public VisionTuner(String[] imageFilenames) {
        cameraFrame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);
        cameraFrame.setFocusable(true);
        cameraFrame.requestFocus();

        loadTestImages(imageFilenames);
    }

    /**
     * Loads the named test image files.
     * Sets testImageFilenames and testImages.
     */
    private void loadTestImages(String[] imageFilenames) {
	testImageFilenames = imageFilenames;
	testImages = new WPIColorImage[testImageFilenames.length];

	for (int i = 0; i < testImageFilenames.length; i++) {
            String imageFilename = testImageFilenames[i];

            System.out.println("Loading image file: " + imageFilename);
            WPIColorImage rawImage = null;
            try {
                rawImage = new WPIColorImage(ImageIO.read(
                	new File(imageFilename)));
            } catch (IOException e) {
                System.err.println("Couldn't load image file: " + imageFilename
                	+ ": " + e.getMessage());
                System.exit(1);
                return;
            }
            testImages[i] = rawImage;
	}
    }

    private void processCurrentImage() {
        WPIColorImage cameraImage = testImages[currentIndex];
        cameraFrame.setTitle(testImageFilenames[currentIndex]);

        WPIImage processedImage = recognizer.processImage(cameraImage);
        cameraFrame.showImage(processedImage.getBufferedImage());
    }

    private void previousImage() {
	if (currentIndex > 0) {
	    --currentIndex;
	}
	processCurrentImage();
    }

    private void nextImage() {
	if (currentIndex + 1 < testImages.length) {
	    ++currentIndex;
	}
	processCurrentImage();
    }

    private void processEvents() {
	KeyEvent e = cameraFrame.waitKey();

	switch (e.getKeyCode()) {
	case KeyEvent.VK_LEFT:
	    previousImage();
	    break;
	case KeyEvent.VK_RIGHT:
	    nextImage();
	    break;
	}
    }

    public static void main(final String[] args) {
        if (args.length == 0) {
            System.err.println("Usage: " + VisionTuner.class.getName()
        	    + " test image filenames...");
            System.exit(1);
        }

        VisionTuner tuner = new VisionTuner(args);
        tuner.processCurrentImage();

        for (;;) {
            tuner.processEvents();
        }
    }

}
