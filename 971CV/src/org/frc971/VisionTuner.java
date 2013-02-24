package org.frc971;

import java.awt.BorderLayout;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyEvent;

import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

import java.io.FileNotFoundException;
import java.io.IOException;

import javax.swing.JButton;
import javax.swing.JLabel;
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
 * @author daniel
 */
public class VisionTuner {
    private Recognizer recognizer = new Recognizer2013();

    private final static Logger LOG = Logger.getLogger(Logger.GLOBAL_LOGGER_NAME);
    
    private final CanvasFrame cameraFrame = new CanvasFrame("Camera");
    private final JPanel panel = new JPanel();
    private final JSlider hueMinSlider = new JSlider();
    private final JSlider hueMaxSlider = new JSlider();
    private final JSlider satMinSlider = new JSlider();
    private final JSlider valMinSlider = new JSlider();
    private final JButton showCalibration = new JButton("Calibrate");
    
    private ResultSender sender = null;

    private int totalFrames = -1; // don't count the first (warm-up) frame
    private double totalMsec;
    private double minMsec = Double.MAX_VALUE;
    private double maxMsec;
    
    private TestImageGetter getter;
    
    private WPIColorImage current;
    
    private String currentWindowTitle;
    
    private boolean debug = false;

    public VisionTuner() {
    	//set logger to log everything
        LOG.setLevel(Level.ALL);
        try {
        	LogHandler handler = new LogHandler("ds_vision.log");
        	TimeFormatter formatter = new TimeFormatter();
            handler.setFormatter(formatter);
            LOG.addHandler(handler);
        }
        catch (FileNotFoundException e) {
        	Messages.warning("Logging initialization failed.");
        }
        
        //initialize result sender
        try {
        	sender = new ResultSender();
        }
        catch (IOException e) {
        	LOG.severe("Server initialization failed: " + e.getMessage() + ". Result reporting disabled.");
        }
        cameraFrame.setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

        cameraFrame.getContentPane().add(panel, BorderLayout.SOUTH);
        panel.setLayout(new GridLayout(0, 1, 0, 0));
        
        showCalibration.setToolTipText("Click here if the system is not finding targets well enough.");
        panel.add(showCalibration);
        showCalibration.addActionListener(new ActionListener() {
        	public void actionPerformed(ActionEvent e) {
        		showCalibrationWindow();
        	}
        });

        LOG.fine("Initial HSV range ["
                + hueMinSlider.getValue() + " .. "
                + hueMaxSlider.getValue() + "] "
                + satMinSlider.getValue() + "+ "
                + valMinSlider.getValue() + "+");
    }
    
    /** Shows a calibration window when the user clicks the Calibrate button. */
    private void showCalibrationWindow() {
    	final CanvasFrame calibrationWindow = new CanvasFrame("Calibration");
    	calibrationWindow.setDefaultCloseOperation(WindowConstants.DISPOSE_ON_CLOSE);
    	
    	final JPanel panel = new JPanel();
    	calibrationWindow.getContentPane().add(panel, BorderLayout.SOUTH);
        panel.setLayout(new GridLayout(3, 3, 0, 0));
        
        hueMinSlider.setToolTipText("minimum HSV hue");
        hueMinSlider.setMaximum(255);
        hueMinSlider.setValue(recognizer.getHueMin());
        panel.add(hueMinSlider);
        
        panel.add(new JLabel("min hue                          max hue")); 

        hueMaxSlider.setToolTipText("maximum HSV hue");
        hueMaxSlider.setMaximum(255);
        hueMaxSlider.setValue(recognizer.getHueMax());
        panel.add(hueMaxSlider);

        satMinSlider.setToolTipText("minimum HSV color saturation");
        satMinSlider.setMaximum(255);
        satMinSlider.setValue(recognizer.getSatMin());
        panel.add(satMinSlider);
        
        panel.add(new JLabel("min saturation  max saturation")); 

        valMinSlider.setToolTipText("minimum HSV brightness value");
        valMinSlider.setMaximum(255);
        valMinSlider.setValue(recognizer.getValMin());
        panel.add(valMinSlider);
        
        panel.add(new JLabel("")); //empty cells can cause problems
        
        final JButton done = new JButton("Done");
        panel.add(done);
        done.addActionListener(new ActionListener() {
        	public void actionPerformed(ActionEvent e) {
        		calibrationWindow.dispose();
        	}
        });
        
        panel.add(new JLabel("")); //empty cells can cause problems
        
        ChangeListener sliderListener = new ChangeListener() {
            @Override
            public void stateChanged(ChangeEvent e) {
                LOG.fine("New HSV range ["
                        + hueMinSlider.getValue() + " .. "
                        + hueMaxSlider.getValue() + "] "
                        + satMinSlider.getValue() + "+ "
                        + valMinSlider.getValue() + "+");
                recognizer.setHSVRange(
                        hueMinSlider.getValue(), hueMaxSlider.getValue(),
                        satMinSlider.getValue(),
                        valMinSlider.getValue());
                if (debug) {
                	processImage(current, null);
                }
            }
        };
        
        hueMinSlider.addChangeListener(sliderListener);
        hueMaxSlider.addChangeListener(sliderListener);
        satMinSlider.addChangeListener(sliderListener);
        valMinSlider.addChangeListener(sliderListener);
        
        calibrationWindow.pack();
  
    }

    /**
     * Loads the named test image files.
     * Sets testImageFilenames and testImages.
     */
    private void processImage(WPIColorImage cameraImage, String title) {
    	current = cameraImage;
    	
    	//set window title if it needs to be changed
    	if (title != null && !title.equals(currentWindowTitle)) {
    		cameraFrame.setTitle(title);
    		currentWindowTitle = title;
    	}

        long startTime = System.nanoTime();
        Target target = recognizer.processImage(cameraImage);
        WPIImage processedImage = target.editedPicture;
        long endTime = System.nanoTime();

        cameraFrame.showImage(processedImage.getBufferedImage());

        double milliseconds = (endTime - startTime) / 1e6;
        if (++totalFrames > 0) {
            totalMsec += milliseconds;
            minMsec = Math.min(minMsec, milliseconds);
            maxMsec = Math.max(maxMsec, milliseconds);
            LOG.fine("The recognizer took " + milliseconds + " ms, " + 
            (1000 * totalFrames / totalMsec) + " fps, %.2f avg");
        }
        
        //send results to atom. (and any connected clients)
        if (sender != null) {
        	sender.send(target.azimuth, target.elevation, target.range);
        }
        
    }

    private void previousImage() {
    	WPIColorImage to_process = getter.GetPrev();
    	if (to_process != null)
    		processImage(to_process, getter.GetName());
    }

    private void nextImage() {
    	WPIColorImage to_process = getter.GetNext();
    	if (to_process != null)
    		processImage(to_process, getter.GetName());
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
            LOG.fine("The recognizer took " + (totalMsec / totalFrames) + "ms avg, " + minMsec +" min,"
                    + maxMsec + " max, " + (1000 * totalFrames / totalMsec) + " fps avg");
            System.exit(0);
        }
    }

    public static void main(final String[] args) {
    	VisionTuner tuner = new VisionTuner();
    	Messages.SetWindow(tuner.cameraFrame);
    	
    	String atomIP = null;
    	try {
    		atomIP = args[0];
    	}
    	catch (ArrayIndexOutOfBoundsException e) {
    		System.out.println("Usage: VisionTuner [atom ip]");
    		System.exit(0);
    	}
    	
        if (Arrays.asList(args).contains("-debug")) {
        	//debug mode has been requested
        	tuner.debug = true;
        	
        	//show debugging windows
        	tuner.recognizer.showIntermediateStages(true);
        	
        	tuner.getter = new TestImageGetter(".");
        	WPIColorImage to_process = tuner.getter.GetNext();
        	if (to_process != null) {
        		tuner.processImage(to_process, tuner.getter.GetName());
        		for (;;) {
        			tuner.processEvents();
        		}
        	}
        	else {
        		LOG.severe("Could not load test images.");
        		Messages.severe("Could not load test images.");
        	}	
        }
        else {
        	try {
        		HTTPClient client = new HTTPClient(atomIP);
        		for (;;) {
            		ImageWithTimestamp to_process = client.GetFrame();
            		if (to_process.image != null) {
            			tuner.processImage(to_process.image, client.GetName());
            			LOG.fine("Captured time: " + Double.toString(to_process.timestamp));
            		}
            	}
        	}
        	catch (IOException e) {
        		LOG.severe("Client initialization failed: " + e.getMessage() + ".");
        		Messages.severe("Client initialization failed: " + e.getMessage() + ".");
        	}
        }
    }

}
