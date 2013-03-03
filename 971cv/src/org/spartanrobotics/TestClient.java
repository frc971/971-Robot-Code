/**
 * 
 */
package org.spartanrobotics;

/**
 * @author daniel
 * 
 */

/** Small thread for running vision code concurrently with debug server. */
public class TestClient implements Runnable {
	
	private String atomIP;
	
	private Thread t;
	
	/** Constructor to set up new thread. */
	public TestClient(String atomIP) {
		t = new Thread(this, "Test Client");
		this.atomIP = atomIP;
		t.start();
	}
	
	/** Simple thread, runs the vision code. */
	public void run() {
		String[] args = {atomIP};
		VisionTuner.main(args);
	}
}
