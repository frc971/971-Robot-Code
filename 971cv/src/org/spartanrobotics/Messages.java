/**
 * 
 */
package org.spartanrobotics;

import javax.swing.JOptionPane;

/**
 * @author daniel
 *
 */

import com.googlecode.javacv.CanvasFrame;

/** Allows other classes to display message boxes bound to main window. */
public class Messages {
	
	private static CanvasFrame main;
	
	/** Constructor
	 * 
	 * @param main is the window your messages will be associated with.
	 */
	public static void SetWindow(CanvasFrame main) {
		Messages.main = main;
	}
	/** Shows a warning message. */
	public static void warning(String message) {
		JOptionPane.showMessageDialog(main, message, "Warning:", JOptionPane.WARNING_MESSAGE);
	}
	
	/** Shows a severe error message. */
	public static void severe(String message) {
		JOptionPane.showMessageDialog(main, message, "Severe:", JOptionPane.ERROR_MESSAGE);
	}
	
	/** Shows an info message. */
	public static void info(String message) {
		JOptionPane.showMessageDialog(main, message);
	}
}
