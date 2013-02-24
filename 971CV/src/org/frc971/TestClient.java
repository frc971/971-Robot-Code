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
	
	private String atomIP;
	
	/** Constructor to set up new thread. */
	public TestClient(String atomIP) {
		super("Test Client");
		this.atomIP = atomIP;
		start();
	}
	
	/** Simple thread, runs the vision code. */
	public void run() {
		String[] args = {atomIP};
		VisionTuner.main(args);
	}
}
