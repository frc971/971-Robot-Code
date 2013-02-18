package org.frc971;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;
import javax.swing.JPanel;
import javax.swing.JSlider;
import javax.swing.WindowConstants;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import com.googlecode.javacv.CanvasFrame;

import edu.wpi.first.wpijavacv.WPIColorImage;
import edu.wpi.first.wpijavacv.WPIImage;

/* REQUIRED JAVA LIBRARIES:
 *   external_jars/
 *     javacpp.jar
 *     javacv-YOUR_OS.jar
 *     javacv.jar
 *     WPIJavaCV.jar
 *
 * REQUIRED NATIVE CODE LIBRARIES ON $PATH:
 *   Program Files/WPIJavaCV/     [for example]
 *     JavaCV_2.2.0/javacv-bin/javacv-YOUR_OS.jar
 *     OpenCV_2.2.0/bin/*
 *
 * The native libraries and javacv-YOUR_OS.jar must match the 32 vs. 64-bit JVM.
 */
/**
 * FRC 2013 vision-target recognizer tuner app.
 *
 * <p>
 * See {@link #processEvents()} for the keystroke commands.
 *
 * @author jerry
 */
public class VisionTuner {
    private String[] testImageFilenames;
    private WPIColorImage[] testImages;
    private int currentIndex = 0;
    private Recognizer recognizer = new Recognizer2013();

    private final CanvasFrame cameraFrame = new CanvasFrame("Camera");
    private final JPanel panel = new JPanel();
    private final JSlider hueMinSlider = new JSlider();
    private final JSlider hueMaxSlider = new JSlider();
    private final JSlider satMinSlider = new JSlider();
    private final JSlider valMinSlider = new JSlider();

    private int totalFrames;
    private double totalMsec;
    private double minMsec = Double.MAX_VALUE;
    private double maxMsec;

    public VisionTuner(String[] imageFilenames) {
        cameraFrame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

        loadTestImages(imageFilenames);
        recognizer.showIntermediateStages(true);

        cameraFrame.getContentPane().add(panel, BorderLayout.SOUTH);
        panel.setLayout(new GridLayout(0, 2, 0, 0));

        ChangeListener sliderListener = new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                System.out.println("New HSV range ["
                        + hueMinSlider.getValue() + " .. "
                        + hueMaxSlider.getValue() + "], ["
                        + satMinSlider.getValue() + " .. 255], ["
                        + valMinSlider.getValue() + " .. 255]");
                recognizer.setHSVRange(
                        hueMinSlider.getValue(), hueMaxSlider.getValue(),
                        satMinSlider.getValue(),
                        valMinSlider.getValue());
                processCurrentImage();
            }
        };

        hueMinSlider.setValue(recognizer.getHueMin());
        hueMinSlider.setToolTipText("minimum HSV hue");
        hueMinSlider.setMaximum(255);
        panel.add(hueMinSlider);

        hueMaxSlider.setValue(recognizer.getHueMax());
        hueMaxSlider.setToolTipText("maximum HSV hue");
        hueMaxSlider.setMaximum(255);
        panel.add(hueMaxSlider);

        satMinSlider.setValue(recognizer.getSatMin());
        satMinSlider.setToolTipText("minimum HSV color saturation");
        satMinSlider.setMaximum(255);
        panel.add(satMinSlider);

        valMinSlider.setValue(recognizer.getValMin());
        valMinSlider.setToolTipText("minimum HSV brightness value");
        valMinSlider.setMaximum(255);
        panel.add(valMinSlider);

        hueMinSlider.addChangeListener(sliderListener);
        hueMaxSlider.addChangeListener(sliderListener);
        satMinSlider.addChangeListener(sliderListener);
        valMinSlider.addChangeListener(sliderListener);
    }

    /**
     * Loads the named test image files.
     * Sets testImageFilenames and testImages.
     */
    private void loadTestImages(String[] imageFilenames) {
        testImageFilenames = imageFilenames;
        testImages = new WPIColorImage[testImageFilenames.length];
        currentIndex = 0;

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

        long startTime = System.nanoTime();
        WPIImage processedImage = recognizer.processImage(cameraImage);
        long endTime = System.nanoTime();

        cameraFrame.showImage(processedImage.getBufferedImage());

        double milliseconds = (endTime - startTime) / 1e6;
        ++totalFrames;
        totalMsec += milliseconds;
        minMsec = Math.min(minMsec, milliseconds);
        maxMsec = Math.max(maxMsec, milliseconds);
        System.out.format("The recognizer took %.2f ms, %.2f fps, %.2f avg%n",
                milliseconds, 1000 / milliseconds,
                1000 * totalFrames / totalMsec);
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
        case KeyEvent.VK_LEFT: // left arrow key: go to previous image
            previousImage();
            break;
        case KeyEvent.VK_RIGHT: // right arrow key: go to next image
            nextImage();
            break;
        case KeyEvent.VK_Q: // Q: print time measurements then quit
            System.out.format("The recognizer took %.2f ms avg, %.2f min,"
                    + " %.2f max, %.2f fps avg%n",
                    totalMsec / totalFrames,
                    minMsec, maxMsec,
                    1000 * totalFrames / totalMsec);
            System.exit(0);
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
