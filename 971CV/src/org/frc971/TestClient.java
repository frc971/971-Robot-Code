/**
 * 
 */
package org.frc971;

/**
 * @author daniel
 * 
 */

/** Small thread for running vision code concurrently with debug server. */
public class TestClient extends Thread {
	
	/** Constructor to set up new thread. */
	public TestClient() {
		super("Test Client");
		start();
	}
	
	/** Simple thread, runs the vision code. */
	public void run() {
		String[] args = {};
		VisionTuner.main(args);
	}
}
